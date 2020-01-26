void setup()
{
  DDRD = 0b11111111;
}

void loop()
{
  int i =0;
  PORTD = 0b00000001;
  delay(1000);
  for(i=0;i<9;i++)
  {
      PORTD = PORTD << 1;
      delay(800);
    }
 }
