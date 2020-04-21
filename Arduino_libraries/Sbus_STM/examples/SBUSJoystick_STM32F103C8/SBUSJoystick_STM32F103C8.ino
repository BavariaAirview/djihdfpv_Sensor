/*
 * This example is for the STM32F103C8 board.
 * Boards manager url: http://dan.drown.org/stm32duino/package_STM32duino_index.json
 * Forked from: Zendes/SBUS By: DarrenLouw
 */

#include <SBUS.h>
#include <USBComposite.h>

USBHID HID;
HIDJoystick Joystick(HID);
SBUS sbus(Serial2);

void setup()
{
  sbus.begin();
  setupTimer2();
  HID.begin(HID_JOYSTICK);
  Joystick.setManualReportMode(true);
}

void loop()
{
    Joystick.Y(map(sbus._channels[2], 172, 1811, 1023, 0));
    Joystick.X(map(sbus._channels[1], 172, 1811, 0, 1023));
    Joystick.Xrotate(map(sbus._channels[0], 172, 1811, 0, 1023));
    Joystick.Yrotate(map(sbus._channels[3], 172, 1811, 0, 1023));
    Joystick.button(1,sbus._channels[4] > 1500);
    Joystick.button(2,sbus._channels[5] > 1500);
    Joystick.send();
}

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
