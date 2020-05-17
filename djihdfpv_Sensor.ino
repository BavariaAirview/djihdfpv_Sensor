/* DJI HD FPV MSP
    Converts Sensordata data to MSP telemetry data compatible with the DJcycle HD FPV system.
    based on d3ngit/djihdfpv_mavlink_to_msp_V2

    HW used: BluePill - STM32

    Arduino TX to DJI Air unit RX(115200)
    DJI SW 01.00.05 required
*/

//#define debug 1
#define RSSI_Servo  1
//#define RSSI_SBUS 1
#define SERIAL_TYPE                                                 1       //0==SoftSerial(Arduino_Nano), 1==HardSerial(Bluepill)
//#define IMPERIAL_UNITS                                                    //Altitude in feet, distance to home in miles.
#define STORE_GPS_LOCATION_IN_SUBTITLE_FILE                                 //comment out to disable. Stores GPS location in the goggles .srt file in place of the "uavBat:" field at a slow rate of ~2-3s per GPS coordinate
#include <MSP.h>
#include "MSP_OSD.h"
#include "OSD_positions_config.h"
#include <TinyGPS++.h>

#define COUNT_Filter 15

String craft_name = "no Name";

int8_t offsetBat = 1000;   //mV
uint8_t scaleBat = 111;


#ifdef RSSI_Servo
#include <Servo.h>
#define CHANNEL_1_PIN PB12    // B12
#endif

#ifdef RSSI_SBUS
#include <SBUS_STM.h>
SBUS sbus(Serial1);         // B
#define RSSI_CH   12
#endif

static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

#define Vbat_in   PA0    // V_Bat input  A0
#define Battery_scaler 330
#define Resoulition 4096

#if SERIAL_TYPE == 0
#include <AltSoftSerial.h>
HardwareSerial &mspSerial = Serial;
AltSoftSerial ss;
#elif SERIAL_TYPE == 1
HardwareSerial &mspSerial = Serial3;
HardwareSerial &ss = Serial2;
#endif
MSP msp;

uint8_t flightModeFlags = 0;
uint32_t previousMillis_MSP = 0;
const uint32_t next_interval_MSP = 100;
String cnameStr = "";
uint16_t batteryCapacity = 1800;
uint8_t vbat = 0;
float groundspeed = 0;
int32_t relative_alt = 0;       // in milimeters
int32_t altitude_msp = 0;      // EstimatedAltitudeCm
uint16_t rssi = 0;
char craftname[15] = "";
float f_mAhDrawn = 0.0;
uint8_t numSat = 0;
uint8_t pid_roll[3];
uint8_t pid_pitch[3];
uint8_t pid_yaw[3];
int32_t gps_lon = 0;
int32_t gps_lat = 0;
int32_t gps_alt = 0;
int32_t gps_home_lon = 0;
int32_t gps_home_lat = 0;
int32_t gps_home_alt = 0;
uint16_t fix_age = 0;
int16_t roll_angle = 0;
int16_t pitch_angle = 0;
uint32_t distanceToHome = 0;    // distance to home in meters
int16_t directionToHome = 0;   // direction to home in degrees
uint8_t fix_type = 0;           // < 0-1: no fix, 2: 2D fix, 3: 3D fix
uint8_t batteryCellCount = 0;
uint8_t legacyBatteryVoltage = 0;
uint8_t batteryState = 0;       // voltage color 0==white, 1==red
uint16_t batteryVoltage = 0;
int16_t heading = 0;
float Vbat = 0;
float Vbat_lowpass[20];
char gps_time[32];
uint16_t fail_timer = 1000;
uint8_t i = 1;
uint16_t lowpass_filter = 0;
uint16_t debugBat = 0;

uint8_t set_home = 1;
uint32_t general_counter = 0;
uint8_t srtCounter = 1;
float climb_rate = 0.0;

volatile unsigned long timer_start;
volatile int last_interrupt_time; //calcSignal is the interrupt handler
int pulse_time = 0;

#ifdef RSSI_Servo
void calcSignal()
{
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  last_interrupt_time = micros();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(CHANNEL_1_PIN) == HIGH)
  {
    timer_start = micros();
    fail_timer = millis();
  }
  //otherwise, the pin has gone LOW
  else
  {
    //only worry about this if the timer has actually started
    if (timer_start != 0)
    {
      //record the pulse time
      pulse_time = ((volatile int)micros() - timer_start);
      //restart the timer
      timer_start = 0;
    }
    if (pulse_time >= 2500) pulse_time = 960;
    if (pulse_time < 960) pulse_time = 960;
  }
}
#endif

void setup()
{
#ifdef debug
  Serial.begin(115200);
#endif
  mspSerial.begin(115200);
  msp.begin(mspSerial);
  pinMode(Vbat_in, INPUT);
  ss.begin(GPSBaud);
  delay(1000);
  timer_start = 0;
#ifdef RSSI_Servo
  attachInterrupt(CHANNEL_1_PIN, calcSignal, CHANGE);
#endif
#ifdef RSSI_SBUS
  sbus.begin();
  setupTimer2();
#endif
}

void loop(){
  
#ifdef debug
  _debug();
#endif
  VoltageBat();

#ifdef RSSI_SBUS
  rssi = map(sbus._channels[RSSI_CH], 172, 1811, 950, 2000);
#endif

#ifdef RSSI_Servo
  get_Servo();
#endif
  //send MSP data
  uint32_t currentMillis_MSP = millis();
  if ((uint32_t)(currentMillis_MSP - previousMillis_MSP) >= next_interval_MSP) {
    previousMillis_MSP = currentMillis_MSP;
    GPS_recieve();
    GPS_calculateDistanceAndDirectionToHome();
    send_msp_to_airunit();

    general_counter += next_interval_MSP;
  }


  if (batteryCellCount == 0 && vbat > 10 && millis() > 5000)set_battery_cells_number();

  //set GPS home when 3D fix
  if (fix_type > 2 && set_home == 1 && gps_lat != 0 && gps_lon != 0 && numSat >= 6) {
    gps_home_lat = gps_lat;
    gps_home_lon = gps_lon;
    gps_home_alt = gps_alt;
    GPS_calc_longitude_scaling(gps_home_lat);
    set_home = 0;
  }
}

#ifdef RSSI_Servo
void get_Servo(){
  rssi = map(pulse_time, 980, 2000, 10, 1050);
  if (rssi > 1050) rssi = 1050;
  if (fail_timer < millis() - 1000) {
    rssi = 0;
  }
}
#endif

#ifdef RSSI_SBUS
void setupTimer2()
{
  Timer2.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
  Timer2.setPeriod(249);
  Timer2.setCompare(TIMER_CH1, 1);
  Timer2.attachInterrupt(TIMER_CH1, sbusProcess);
}

void sbusProcess()
{
  sbus.process();
}
#endif

#ifdef debug
void _debug()
{
  Serial.print("Cycle    ");
  Serial.println (i);
  Serial.print("Fix age  ");
  Serial.println (fix_age);
  Serial.print("Vbat:    ");
  Serial.println (vbat);
  Serial.print("batIN:    ");
  Serial.println (debugBat);
  Serial.print("Num Cell:");
  Serial.println (batteryCellCount);
  Serial.print("Sat Fix  ");
  Serial.println (fix_type);
  Serial.print("Num Sat  ");
  Serial.println (numSat);
  Serial.print("lat      ");
  Serial.println (gps_lat/  10000000);
  Serial.print("lon      ");
  Serial.println (gps_lon/  10000000);
  Serial.print("Alt_rel  ");
  Serial.println (relative_alt);
  Serial.print("GPS alt  ");
  Serial.println (gps_alt,1);
  Serial.print("Home alt ");
  Serial.println (gps_home_alt,1);
  Serial.print("HDG      ");
  Serial.println (heading);
  Serial.print("SPEED    ");
  Serial.println (groundspeed / 100);
  Serial.print("RSSI     ");
  Serial.println (rssi);
  Serial.println("");
}
#endif

void GPS_recieve()
{
  smartDelay(150);

  numSat = gps.satellites.value();
  gps_lat = gps.location.lat() * 10000000;
  gps_lon = gps.location.lng() * 10000000;
  gps_alt = gps.altitude.meters() * 100;
  fix_age = gps.location.age();
  heading = gps.course.deg();
  groundspeed = gps.speed.kmph() * 100;

  if (fix_age != 0 && fix_age < 2000 && numSat >= 6) {
    fix_type = 3;
  }
  if (set_home == 0) {
    relative_alt = gps_alt - gps_home_alt;
    altitude_msp = relative_alt;
  } else {
    altitude_msp = gps_alt;
  }
  if ( fix_type == 3 && flightModeFlags == 1) {
    flightModeFlags = 2;
  }
  if ( fix_type == 3 && flightModeFlags == 0) {
    flightModeFlags = 1;
    //     <entry value="0x00000001" name="AQ_NAV_STATUS_STANDBY">
    //     <entry value="0x00000002" name="AQ_NAV_STATUS_MANUAL">
  }

}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void VoltageBat()
{
  Vbat = analogRead(Vbat_in);
  Vbat = Vbat / Resoulition * 100;
  Vbat = Vbat * Battery_scaler / 100;
  //Serial.println(Vbat);
  Vbat_lowpass[i] = Vbat;
  i++;
  if (i > COUNT_Filter) i = 1;
  for (int j = 1; j <= COUNT_Filter; j++) {
    lowpass_filter = lowpass_filter + Vbat_lowpass[j];
  }

  vbat = lowpass_filter / COUNT_Filter;
  lowpass_filter = 0;
  debugBat = vbat;
  vbat = (vbat * scaleBat / 100) + (offsetBat / 100);

}

void set_battery_cells_number()
{
  if     (vbat < 127)batteryCellCount = 3;
  else if (vbat < 169)batteryCellCount = 4;
  else if (vbat < 211)batteryCellCount = 5;
  else if (vbat < 255)batteryCellCount = 6;
}

void save_text(char (*text)[15])
{
  memcpy(craftname, *text, sizeof(craftname));
}


msp_battery_state_t battery_state = {0};
msp_name_t name = {0};
//msp_fc_version_t fc_version = {0};
msp_status_BF_t status_BF = {0};
msp_analog_t analog = {0};
msp_raw_gps_t raw_gps = {0};
msp_comp_gps_t comp_gps = {0};
msp_attitude_t attitude = {0};
msp_altitude_t altitude = {0};

void send_msp_to_airunit()
{

  //MSP_FC_VERSION
  // fc_version.versionMajor = 4;
  // fc_version.versionMinor = 1;
  // fc_version.versionPatchLevel = 1;
  // msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));

  cnameStr.toCharArray(name.craft_name, sizeof(craftname));
  msp.send(MSP_NAME, &name, sizeof(name));

  //MSP_STATUS
  status_BF.flightModeFlags = flightModeFlags;
  msp.send(MSP_STATUS, &status_BF, sizeof(status_BF));

  //MSP_ANALOG
  analog.vbat = vbat;
  analog.rssi = rssi;
  msp.send(MSP_ANALOG, &analog, sizeof(analog));

  //MSP_BATTERY_STATE
  battery_state.batteryVoltage = vbat * 10;
  battery_state.batteryCellCount = batteryCellCount;
  battery_state.batteryCapacity = batteryCapacity;
  battery_state.batteryState = batteryState;

#ifdef STORE_GPS_LOCATION_IN_SUBTITLE_FILE
  if (general_counter % 400 == 0 && srtCounter != 0) {
    String str = "";
    str = str + (abs(gps_lat) / 10) + "" + (abs(gps_lon) / 10);
    char m[] = {str[srtCounter - 1], str[srtCounter]};
    battery_state.legacyBatteryVoltage = atoi(m);
    if (srtCounter <= (str.length() - 2))srtCounter += 2;
    else srtCounter = 0;
  }
  else if (general_counter % 400 == 0 && srtCounter < 1) {
    battery_state.legacyBatteryVoltage = 255;
    srtCounter = 1;
  }
#else
  battery_state.legacyBatteryVoltage = vbat;
#endif
  msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));

  //MSP_RAW_GPS
  raw_gps.lat = gps_lat;
  raw_gps.lon = gps_lon;
  raw_gps.numSat = numSat;
  raw_gps.alt = (int16_t)altitude_msp;
  raw_gps.groundSpeed = (int16_t)groundspeed / 3.6;
  msp.send(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps));

  //MSP_COMP_GPS
  uint16_t directionToHomeNorm = 360 + (directionToHome - heading);
    if(directionToHomeNorm < 0)directionToHomeNorm += 360;
    if(directionToHomeNorm > 360)directionToHomeNorm -= 360;
  comp_gps.distanceToHome = (int16_t)distanceToHome;
  comp_gps.directionToHome = directionToHomeNorm;

  msp.send(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps));

  //MSP_ATTITUDE
  attitude.pitch = pitch_angle;
  attitude.roll = roll_angle;
  msp.send(MSP_ATTITUDE, &attitude, sizeof(attitude));

  //MSP_ALTITUDE
  altitude.estimatedActualPosition = altitude_msp; //cm
  altitude.estimatedActualVelocity = (int16_t)(climb_rate * 100); //m/s to cm/s
  msp.send(MSP_ALTITUDE, &altitude, sizeof(altitude));

  //MSP_OSD_CONFIG
  send_osd_config();
}

msp_osd_config_t msp_osd_config = {0};

void send_osd_config()
{

#ifdef IMPERIAL_UNITS
  msp_osd_config.units = 0;
#else
  msp_osd_config.units = 1;
#endif

  msp_osd_config.osd_item_count = 56;
  msp_osd_config.osd_stat_count = 24;
  msp_osd_config.osd_timer_count = 2;
  msp_osd_config.osd_warning_count = 16;              // 16
  msp_osd_config.osd_profile_count = 1;              // 1
  msp_osd_config.osdprofileindex = 1;                // 1
  msp_osd_config.overlay_radio_mode = 0;             //  0

  msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
  msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
  msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
  msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
  msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
  msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
  msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
  msp_osd_config.osd_flymode_pos = osd_flymode_pos;
  msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
  msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
  msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
  msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
  msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
  msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
  msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
  msp_osd_config.osd_altitude_pos = osd_altitude_pos;
  msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
  msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
  msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
  msp_osd_config.osd_power_pos = osd_power_pos;
  msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
  msp_osd_config.osd_warnings_pos = osd_warnings_pos;
  msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
  msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
  msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
  msp_osd_config.osd_debug_pos = osd_debug_pos;
  msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
  msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
  msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
  msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
  msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
  msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
  msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
  msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
  msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
  msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
  msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
  msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
  msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
  msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
  msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
  msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
  msp_osd_config.osd_g_force_pos = osd_g_force_pos;
  msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
  msp_osd_config.osd_log_status_pos = osd_log_status_pos;
  msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
  msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
  msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
  msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
  msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
  msp_osd_config.osd_display_name_pos = osd_display_name_pos;
  msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
  msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
  msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
  msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
  msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
  msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;

  msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}

void invert_pos(uint16_t *pos1, uint16_t *pos2)
{
  uint16_t tmp_pos = *pos1;
  *pos1 = *pos2;
  *pos2 = tmp_pos;
}



//from here on code from Betaflight github https://github.com/betaflight/betaflight/blob/c8b5edb415c33916c91a7ccc8bd19c7276540cd1/src/main/io/gps.c

float GPS_scaleLonDown = 1.0f;  // this is used to offset the shrinking longitude as we go towards the poles

void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (fabsf((float)lat) / 10000000.0f) * 0.0174532925f;
    GPS_scaleLonDown = cos_approx(rads);
}

#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS 1.113195f
#define TAN_89_99_DEGREES 5729.57795f
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing)
{
    float dLat = *destinationLat2 - *currentLat1; // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(*destinationLon2 - *currentLon1) * GPS_scaleLonDown;
    *dist = sqrtf(sq(dLat) + sq(dLon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS;

    *bearing = 9000.0f + atan2f(-dLat, dLon) * TAN_89_99_DEGREES;      // Convert the output radians to 100xdeg
    if (*bearing < 0)
        *bearing += 36000;
}

void GPS_calculateDistanceAndDirectionToHome(void)
{
     if (gps_home_lat != 0 && gps_home_lon != 0) {      // If we don't have home set, do not display anything
        uint32_t dist;
        int32_t dir;
        GPS_distance_cm_bearing(&gps_lat, &gps_lon, &gps_home_lat, &gps_home_lon, &dist, &dir);
        distanceToHome = dist / 100;
        directionToHome = dir / 100;
     } else {
         distanceToHome = 0;
         directionToHome = 0;
     }
}

#define M_PIf       3.14159265358979323846f
#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0

float sin_approx(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32) return 0.0f;                               // Stop here on error input (5 * 360 Deg)
    while (x >  M_PIf) x -= (2.0f * M_PIf);                                 // always wrap input angle to -PI..PI
    while (x < -M_PIf) x += (2.0f * M_PIf);
    if (x >  (0.5f * M_PIf)) {
      x =  (0.5f * M_PIf) - ( x - (0.5f * M_PIf));   // We just pick -90..+90 Degree
    }
    else if (x < -(0.5f * M_PIf)){
      x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
    }
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
    return sin_approx(x + (0.5f * M_PIf));
}
