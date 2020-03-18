#define DATA_PIN 10
#define CLOCK_PIN 11
#define SS_PIN 12

int map7seg[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7d, 0x07, 0x7F, 0x6F};
int count = 0;

void spi_write(unsigned char cData)
{
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, cData);
  digitalWrite(SS_PIN, HIGH);
  digitalWrite(SS_PIN, LOW);
}

int CalOCR(double freq, long prescaler)
{
  const long CLK_FREQ = 16000000;
  return (CLK_FREQ/(freq*prescaler))-1.0;
}

void setup()
{
  pinMode(SS_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = CalOCR(1, 1024);
  TCCR1B |= (1<<WGM12);
  TCCR1B |= (1<<CS12)|(1<<CS10); 
  TIMSK1 |= (1<<OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect)
{
  spi_write(map7seg[count-1]);
}

void loop()
{
  if(count>9){ 
  count = 0;
  }
  count++;
  delay(1000);
}
