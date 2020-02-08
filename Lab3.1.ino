int map7seg[16] = {0x3F, 0x06, 0x5B,0x4F,0x66, 0x6D,0x7d,0x07,0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E,0x79, 0x71};
int clockpin = 4;
int datapin = 5;
int latchpin = 3;
void setLED(uint8_t state){
  digitalWrite(latchpin, LOW);
  for(int i = 0;i<8;i++){
    digitalWrite(datapin,((state>>(7-i)) & 0x01));
    digitalWrite(clockpin, HIGH);
    digitalWrite(clockpin, LOW);
    }
    digitalWrite(latchpin, HIGH);   
}

void setup()
{
  DDRD = 0xFF;
  pinMode(clockpin, OUTPUT);
  pinMode(latchpin, OUTPUT);
  pinMode(datapin, OUTPUT);
  
}

void loop()
{
  delay(1000);
  
  for(int i =0; i<16;i++){
    setLED(map7seg[i]);
    delay(1000);
  }
}
