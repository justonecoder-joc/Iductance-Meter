#include <Arduino.h>
#include <stdint.h>

void setup()
{
  Serial.begin(9600);
  // sei();
  // COM1B1:0 = 0b10. Fast PWM mode.
  // WGM13:0 = 0b1111. Fast PWM.
  // ICES1 = 0b0, in TCCR1B. Input capture triggers on falling egde.
  // Fast PWM, no clock prescaler, set OC1B on BOTTOM, clear on compare match with OCR1B.
  // DDRB |= (1 << PB2); // OC1B
  DDRB |= (1 << PB1); // OC1A
  
  // TIMSK1 |= (1 << ICIE1); // Input capture interrupt enable. 

  OCR1A = 719; // Over flow at this value.
  // OCR1AH = 0xFF;
  // OCR1AL = 0xFF;

  // OCR1B = 720; // Set OC1B at this value to get a pulse width of 45 us at tick period of 62.5 ns.
  // OCR1BH = 0x02;
  // OCR1BL = 0xD0;

  // TCCR1A |= (1 << COM1B1) | (1 << WGM11) | (1 << WGM10);
  // TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  TCCR1A |= (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
  TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  // TIFR
  // TIMSK1 |= (1 << OCIE1B) | (1 << OCIE1A) | (1 << TOIE1);
  // TIMSK1 |= (1 << TOIE1);
}

void loop()
{

}

ISR(TIMER1_COMPA_vect)
{
  Serial.print("Timer 1 compare match with OCR1A!, TCNT1 = ");
  Serial.println(TCNT1);
}

ISR(TIMER1_COMPB_vect)
{
  Serial.print("Timer 1 compare match with OCR1B!, TCNT1 = ");
  Serial.println(TCNT1);
}

ISR(TIMER1_OVF_vect)
{
  uint16_t ocr1a_val = OCR1AH << 8;
  ocr1a_val += OCR1AL;
  
  uint16_t ocr1b_val = OCR1BL;
  ocr1b_val += OCR1BH << 8;

  Serial.println("Timer 1 Overflow occurred!, TCNT1 = ");
  Serial.println(ocr1a_val, HEX);
  Serial.println(ocr1b_val, HEX);
  Serial.println(TCCR1A, BIN);
}