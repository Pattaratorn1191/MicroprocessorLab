int map7seg[16] = {0x3F, 0x06, 0x5B,0x4F,0x66, 0x6D,0x7d,0x07,0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E,0x79, 0x71};
int clockpin = 2;
int datapin = 4;
int latchpin = 3;
int c = 0;

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
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  Serial.begin(9600);
  
}

void loop()
{
  int pinA  = digitalRead(A0);
  int pinB = digitalRead(A1);
  
  Serial.println(pinA);
  Serial.println(pinB);
  Serial.println("-----------");

  if(pinA==0 && pinB ==1){
    Serial.println(map7seg[c]);
    c++;
    
  }
  if(pinA == 1 && pinB == 0){
    Serial.println(map7seg[c]);
    c--;
    
  }
  if (c<0)
    c=16;
  if (c>16)
    c=0;
  setLED(map7seg[c]);
  delay(1000);
  
  
}
