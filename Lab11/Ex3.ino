
#define MOTOR_D1_PIN 7
#define MOTOR_D2_PIN 8
#define MOTOR_PWM_PIN 6

int interruptChannelAPin = 2;
int interruptChannelBPin = 3;
int delay_count = 0;

volatile int encoderCount = 0;
volatile int encoderDir = 0;

void moveForward(int speed)
{
  digitalWrite(MOTOR_D1_PIN,HIGH);
  digitalWrite(MOTOR_D2_PIN,LOW);
  analogWrite(MOTOR_PWM_PIN,speed);
}

void moveBackward(int speed)
{
  digitalWrite(MOTOR_D1_PIN,LOW);
  digitalWrite(MOTOR_D2_PIN,HIGH);
  analogWrite(MOTOR_PWM_PIN,speed);
}

void setSpeed(int speed)
{
  if(speed>0)
  {
    if(speed>255)
    {
     speed = 255; 
    }
    moveForward(speed);
  }

  else if(speed<0)
  {
    speed = (-1)*speed;
    if(speed<0)
    {
     speed = 255; 
    }
    moveBackward(speed);
  }
  else
  {
    moveForward(1);
  }
}

void setup()
{
  pinMode(A5,INPUT);
  pinMode(MOTOR_D1_PIN,OUTPUT);
  pinMode(MOTOR_D2_PIN,OUTPUT);
  pinMode(MOTOR_PWM_PIN,OUTPUT);
  
  pinMode(interruptChannelAPin,INPUT_PULLUP);
  pinMode(interruptChannelBPin,INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(interruptChannelAPin),ChannelA_callback,RISING);
  attachInterrupt(digitalPinToInterrupt(interruptChannelBPin),ChannelB_callback,RISING);
  
  Serial.begin(9600);
}

void loop()
{
  int setpoint = analogRead(A5);
  int error = setpoint - encoderCount;
  int p = 1.1*(float)(error);
  setSpeed(p);
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(encoderCount);
  delay(100);
}

void ChannelA_callback()
{
  if(digitalRead(interruptChannelAPin)== 1 && digitalRead(interruptChannelBPin)== 0)
  {
    encoderCount++;
    encoderDir = 1;
  }
}

void ChannelB_callback()
{
  if(digitalRead(interruptChannelAPin)== 0 && digitalRead(interruptChannelBPin)== 1)
  {
    encoderCount--;
    encoderDir = 0;
  }
}
