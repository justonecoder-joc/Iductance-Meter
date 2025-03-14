#define COM_PIN PD7 // Arduino pin 7. AIN1
// #define COM_REF PD6 // Arduino pin 6. AIN0

#define PULSE_OUT PB2 // OC1B. Arduino pin 10.
#define TIMER_F 2E6
#define PULSE_45US uint16_t(45E-6 * TIMER_F - 1)

// float TIMER_PERIOD = 1E6 / TIMER_F;
float TIMER_PERIOD = 0.5f;

uint32_t toggle_stamp_fall = 0;
uint32_t toggle_stamp_rise = 0;
uint32_t cycle = 0;
uint8_t flag = false;

void setup()
{
  cli();
  
  // Reset Arduino IDE Timer1 presets.
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  Serial.begin(9600);

  DDRB |= (1 << PB2);

  OCR1A = 65535; // Over flow. Set OC1B on next clock cycle.
  OCR1B = PULSE_45US; // Clear OC1B. After 45 us.
  
  // Set WGM13:0 = 0b1111. Fast PWM.
  TCCR1A |= (1 << WGM11) | (1 << WGM10);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // COM1B1:0 = 0b10. Clear on compare match, set at BOTTOM.
  TCCR1A |= (1 << COM1B1);

  // Configure Input capture.
  // TCCR1B &= ~(1 << ICES1); // Falling edge capture. 
  TCCR1B |= (1 << ICES1); // Rising edge capture. 
  TIMSK1 |= (1 << ICIE1); // Enable input capture interrupt.

  // Output compare interrupt enable to increase timer resolution.
  // TIMSK1 |= (1 << TOIE1);

  // When AIN1 is LOW, comparator output is HIGH. Therefore, the edge detection of input capture should be flipped.
  // Analog Comparator configuration.
  ADCSRB &= ~(1 << ACME); // AIN1 is applied to the negative input of the analog comparator.
  ACSR = 0;
  ACSR |= (1 << ACBG) | (1 << ACIC); // Bandgap reference, and input capture.
  TIFR1 |= (1 << ICF1); // Clear input capture flag that was set by changing the input capture source.

  sei();// Enable Global Interrupts
  
  
  // Timer1 start.
  // CLK / 8 prescaler.
  TCCR1B &= ~((1 << CS12) | (1 << CS10));
  TCCR1B |= (1 << CS11);

  // // CLK / 256 prescaler. No Prescaler.
  // TCCR1B &= ~((1 << CS11) | (1 << CS10));
  // TCCR1B |= (1 << CS12);

  // Write high byte first, because writing low byte causes timer write.
  // OCR1AH = COMP_VAL >> 8;
  // OCR1AL = COMP_VAL;
}

void loop()
{
  if (flag)
  {
    Serial.print("timestamp fall = ");
    Serial.print(toggle_stamp_fall);

    Serial.print(", timestamp rise = ");
    Serial.print(toggle_stamp_rise);
    
    double period = double(toggle_stamp_fall - toggle_stamp_rise);

    Serial.print(", period = ");
    Serial.print(period);
    Serial.print(" us, Inductance = ");
    Serial.print((period * period) / (4 * PI * PI * 19));
    Serial.println(" uH");

    // cli();
    flag = true;
    cycle = 0;

    TIFR1 |= (1 << ICF1);
    sei();
  }
}

ISR(TIMER1_CAPT_vect)
{
  if (cycle == 3)
  {
    flag = true;
    cli();
  }
  if (TCCR1B & (1 << ICES1)) // Trigger on rising comparator edge. Falling input edge.
  {
    toggle_stamp_fall = ICR1;
    TCCR1B &= ~(1 << ICES1);
    cycle++;
  }
  else // Trigger on Falling comparator edge. Rising input edge.
  {
    toggle_stamp_rise = ICR1;
    TCCR1B |= (1 << ICES1);
  }
}
