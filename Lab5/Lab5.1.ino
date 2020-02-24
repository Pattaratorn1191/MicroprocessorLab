#include <ThreeWire.h>
#include <RtcDS1302.h>
#define ledpin 3
ThreeWire myWire(4,5,2);
RtcDS1302<ThreeWire>Rtc(myWire);
void printDateTime(const RtcDateTime& dt){
  Serial.print(dt.Year(),DEC);
  Serial.print('/');
  Serial.print(dt.Month(),DEC);
  Serial.print('/');
  Serial.print(dt.Day(),DEC);
  Serial.print(' ');
  Serial.print(dt.Hour(),DEC);
  Serial.print(':');
  Serial.print(dt.Minute(),DEC);
  Serial.print(':');
  Serial.print(dt.Second(),DEC);
  Serial.print(' ');
  Serial.print("Total Seconds since 1/1/2000: ");
  Serial.print(dt.TotalSeconds());
  Serial.println();
}
void setup() {
  pinMode(ledpin, OUTPUT);
  Serial.begin(57600);
  Rtc.Begin();
  RtcDateTime setting_time(2020,2,2,23,31,10);
  printDateTime(setting_time);
  Rtc.SetDateTime(setting_time);

}

void loop() {
  RtcDateTime now = Rtc.GetDateTime();
  printDateTime(now);
  delay(500);


}
