#define LED_PIN 12
#define INT_PIN 3
volatile int x = 1;

int CalOCR(double freq, long prescaler)
{
  const long CLK_FREQ = 16000000;
  return (CLK_FREQ/(freq*prescaler))-1.0;
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(INT_PIN, INPUT_PULLUP);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = CalOCR(1, 1024);
  TCCR1B |= (1<<WGM12);
  TCCR1B |= (1<<CS12)|(1<<CS10); 
  TIMSK1 |= (1<<OCIE1A);
  interrupts();

  attachInterrupt(digitalPinToInterrupt(INT_PIN),Button,RISING);
}

void Button()
{
  digitalWrite(LED_PIN,HIGH);
  x = x>= 3 ? 1:x+1;
  OCR1A = CalOCR(1.0/(double)(x), 1024);
}

ISR(TIMER1_COMPA_vect)
{
  PORTB ^= (1<<5);
}

void loop()
{
  delay(1000);
}
