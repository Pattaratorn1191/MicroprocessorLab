int clk_pin = 13;//SHCP
int latch_pin = 10;//STCP
int data_pin = 11;//DS
bool toggle;
int i=0;
int count=0;

void setLED(uint16_t state){
   digitalWrite(latch_pin,LOW);
   for(i = 0 ; i < 16 ; i++){
    digitalWrite(data_pin,(state>>(15-i)) & 0x0001);
    digitalWrite(clk_pin,HIGH);
    digitalWrite(clk_pin,LOW);
   }
   digitalWrite(latch_pin,HIGH);
}
void setup() {
  // put your setup code here, to run once:
  pinMode(clk_pin,OUTPUT);
  pinMode(latch_pin,OUTPUT);
  pinMode(data_pin,OUTPUT);
  }
  
void loop() {
  // put your main code here, to run repeatedly:
  setLED(0b00000000001 << count);
  count++;
  if(count > 15) count = 0;
  
  delay(1000);
  }
