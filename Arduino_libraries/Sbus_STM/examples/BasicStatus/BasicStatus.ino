#include <SBUS.h>
SBUS sbus(Serial3);

void setup()
{
  //Setup timer
  noInterrupts();
  TCCR2A  = 0;
  TCCR2B  = 0;
  TCNT2   = 0;
  OCR2A   = 249;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22);
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
  
  sbus.begin();
  
  Serial.begin(115200);
  Serial.println("SBUS Status");
}

// this is timer2, which triggers ever 1ms and processes the incoming SBUS datastream
ISR(TIMER2_COMPA_vect)
{
  sbus.process();
}

void loop()
{
  delay(300);
  printSBUSStatus();
}

void printSBUSStatus()
{
  Serial.print("Ch1  ");
  Serial.println(sbus._channels[1]);
  Serial.print("Ch2  ");
  Serial.println(sbus._channels[2]);
  Serial.print("Ch3  ");
  Serial.println(sbus._channels[3]);
  Serial.print("Ch4  ");
  Serial.println(sbus._channels[4]);
  Serial.print("Ch5  ");
  Serial.println(sbus._channels[5]);
  Serial.print("Ch6  ");
  Serial.println(sbus._channels[6]);
  Serial.print("Ch7  ");
  Serial.println(sbus._channels[7]);
  Serial.print("Ch8  ");
  Serial.println(sbus._channels[8]);
  Serial.print("Ch9  ");
  Serial.println(sbus._channels[9]);
  Serial.print("Ch10 ");
  Serial.println(sbus._channels[10]);
  Serial.print("Ch11 ");
  Serial.println(sbus._channels[11]);
  Serial.print("Ch12 ");
  Serial.println(sbus._channels[12]);
  Serial.print("Ch13 ");
  Serial.println(sbus._channels[13]);
  Serial.print("Ch14 ");
  Serial.println(sbus._channels[14]);
  Serial.print("Ch15 ");
  Serial.println(sbus._channels[15]);
  Serial.print("Ch16 ");
  Serial.println(sbus._channels[16]);
  Serial.println();
}
