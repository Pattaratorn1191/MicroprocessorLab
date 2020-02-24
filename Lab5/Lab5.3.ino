#include <ThreeWire.h>
#include <RtcDS1302.h>
#define ledpin 3
long previous_time;
ThreeWire myWire(4,5,2);
RtcDS1302<ThreeWire>Rtc(myWire);
void printDateTime(const RtcDateTime &dt){
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
  Serial.print("Total Seconds since 1/1/2020: ");
  Serial.print(dt.TotalSeconds());
  Serial.println();
}
void setup() {

  pinMode(ledpin, OUTPUT);
  Serial.begin(9600);
  Rtc.Begin();
  RtcDateTime setting_time(2020,2,2,23,31,10);
  printDateTime(setting_time);
  Rtc.SetDateTime(setting_time);

  RtcDateTime now = Rtc.GetDateTime();
  previous_time = now.TotalSeconds();

}
void loop() {
  Serial.println("Start");
  RtcDateTime now = Rtc.GetDateTime();
    for(int i = 0;i<60;i++){
      digitalWrite(ledpin, HIGH);
      delay(1000);
      digitalWrite(ledpin, LOW);
      delay(1000);
    }
 printDateTime(now);
  Serial.println("Stop");

  
}
