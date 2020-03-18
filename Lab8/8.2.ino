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
  pinMode(A0, INPUT);
  Serial.begin(9600);

}
void loop() {
 int val = analogRead(A0);
 char num = Serial.read();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print(num);
    display.display();
    Serial.print(val);
  
}

}
