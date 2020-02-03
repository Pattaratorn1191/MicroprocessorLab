int clk_pin = 13;//SHCP
int latch_pin = 10;//STCP
int data_pin = 11;//DS
bool toggle;
int i=0;

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
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
    Serial.begin(9600);
  }
  
void loop() {
  // put your main code here, to run repeatedly:
  int pinA = analogRead(A1);
  int pinB=analogRead(A0);

  int count = 0;
  if(count > 15) count = 0;
  if(count < 0) count = 16; 
 
  if (pinA < 100 && pinB > 100)
    for(int i=0; i<16; i++){
  setLED(0x0001 << count);
  count++;

  
  delay(500);} 
 
   
  if(pinA>100 && pinB < 100)
     for(int i=0; i<16; i++){
    setLED(0x8000 >> count);
  count++;
 
  
  delay(500); }
  }

  
  
