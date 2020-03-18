//Connect 1 LED and 1 Push button switch for selecting mode of LED blinking

#define LED_PIN 1
#define INT_PIN 3
int count = 0;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN),
                  ISR_Callback,FALLING);
}

void loop()
{
  if(count==1)
  {
  digitalWrite(LED_PIN, HIGH);
  delay(1000); 
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  }
  
  else if(count==2)
  {
  digitalWrite(LED_PIN, HIGH);
  delay(2000); 
  digitalWrite(LED_PIN, LOW);
  delay(2000);
  }
  
  else if(count==3)
  {
  digitalWrite(LED_PIN, HIGH);
  delay(3000); 
  digitalWrite(LED_PIN, LOW);
  delay(3000);
  }
}
void ISR_Callback()
{
  count ++;
}
