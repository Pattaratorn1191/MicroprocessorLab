void setup()
{
  DDRD = 0b11111111;
  DDRB = 0b00000000;
}

void loop()
{
  int i = 0;
  PORTD = 0b00000001;
  
  while(i<=7)
  {
    if(PINB == 0b00000001)
    {
      PORTD = PORTD << 1;
      i++;
      delay(300);
    }
  }
}
