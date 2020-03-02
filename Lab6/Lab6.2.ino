
#include <ThreeWire.h>
#include <RtcDS1302.h>

#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH,SCREEN_HEIGHT,&Wire,OLED_RESET);

ThreeWire myWire(9,8,10);
RtcDS1302<ThreeWire>Rtc(myWire);

void printDateTime(const RtcDateTime &dt){
  display.print(dt.Year(),DEC);
  display.print('/');
  display.print(dt.Month(),DEC);
  display.print('/');
  display.print(dt.Day(),DEC);
  display.print(' ');
  display.print(dt.Hour(),DEC);
  display.print(':');
  display.print(dt.Minute(),DEC);
  display.print(':');
  display.print(dt.Second(),DEC);
  display.print(' ');
}
void setup() {
  Serial.begin(9600);
  Rtc.Begin();
  RtcDateTime setting_time(2020,2,2,23,31,10);
  printDateTime(setting_time);
  Rtc.SetDateTime(setting_time);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  display.clearDisplay();
  display.drawPixel(127,63,WHITE);
  

}

void loop() {
  RtcDateTime now = Rtc.GetDateTime();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  printDateTime(now);
  display.display();
}
