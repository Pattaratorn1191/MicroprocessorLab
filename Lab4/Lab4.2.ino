#include <SPI.h>
#define DATA_PIN 11
#define CLOCK_PIN 9
#define SS_PIN_1 10
#define SS_PIN_2 8
int map7seg[10] = {0x3F, 0x06, 0x5B,0x4F,0x66, 0x6D,0x7d,0x07,0x7F, 0x6F};
int x = 0;

void spi_write_1(unsigned char cData) {
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, cData);
  digitalWrite(SS_PIN_1, HIGH);
  digitalWrite(SS_PIN_1, LOW);
}
void spi_write_2(unsigned char cData) {
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, cData);
  digitalWrite(SS_PIN_2, HIGH);
  digitalWrite(SS_PIN_2, LOW);
}
void setup()
{
  pinMode(SS_PIN_1, OUTPUT);
  pinMode(SS_PIN_2, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(12, OUTPUT);
  Serial.begin(9600);
  
}
void show_num(int x)
{
  int ten,one;
  one = x%10;
  ten = x/10;
  spi_write_2(map7seg[one]);
  spi_write_1(map7seg[ten]);
}
void loop()
{
  int pinA  = digitalRead(A0);
  int pinB = digitalRead(A1);
  int pinC = digitalRead(A2);
  
  if(pinC == 0 && pinA == 1 && pinB ==1){
    x = 0;
    tone(12, 1000);
    delay(500);
  noTone(12);
  }
  
  if(pinA==0 && pinB ==1){
    
    x++;
    
  }
  if(pinA == 1 && pinB == 0){
    
    x--;
    }
  show_num(x);
  delay(500);
    
 }
