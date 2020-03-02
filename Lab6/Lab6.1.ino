#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1


Adafruit_SSD1306 display(SCREEN_WIDTH,SCREEN_HEIGHT,&Wire,OLED_RESET);
void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  display.clearDisplay();
  display.drawPixel(127,63,WHITE);
  display.fillCircle(45,50,8,WHITE);
  display.fillCircle(45,30,8,WHITE);
  display.display();
  pinMode(11 ,INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP); 
  Serial.begin(9600);
}

void loop() {
  int sw1 = digitalRead(11);
  int sw2 = digitalRead(12);
  
  if(sw2 ==0 && sw1==1){
     display.clearDisplay(); 
    display.drawCircle(45,50,8,WHITE);
    display.fillCircle(45,30,8,WHITE);
    display.display();
  }
  if(sw1 == 0&&sw2 == 1){
    display.clearDisplay(); 
    display.fillCircle(45,50,8,WHITE);
    display.drawCircle(45,30,8,WHITE);
    display.display();
  }
  

}
