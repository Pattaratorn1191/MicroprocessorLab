#define LED_PIN_1 10
#define LED_PIN_2 11
float brightness = 0;
String buff;
int state;
int pin = 0;
void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()>0){
    char inByte = Serial.read();
    if(inByte == 'L'){
      buff = ' ';
      state = 0;
    }
    if(inByte == '1' && (state == 0)){
     pin = LED_PIN_1;
    }
    if(inByte == '2' && (state == 0)){
     pin = LED_PIN_2;
    }
    if(inByte == 'B'){
      buff = ' ';
      state = 1;
    }
      if(('0'<=inByte)&&(inByte<='9')&& (state == 1)){
        buff += inByte;
    }
      if(inByte == '\n'&& state == 1){
        brightness = buff.toInt();
        Serial.println(brightness);
        analogWrite(pin,int(brightness*2.54));
      }   
    }
  
    
  
}
