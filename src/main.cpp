#include <Arduino.h>

#define COM_PIN PD7 // Arduino pin 7. AIN1
// #define COM_REF PD6 // Arduino pin 6. AIN0

#define PULSE_OUT PB2 // OC1B. Arduino pin 10.
#define TIMER_F 16E6
#define PULSE_45US uint16_t(45E-6 * TIMER_F - 1)
#define CAP_VALUE_UF 106.2f // 69 nF
#define CAP_INV 9.41619f

float L_CONSTANT = 4 * PI * PI / CAP_INV;

uint32_t toggle_stamp_fall_1= 0;
uint32_t toggle_stamp_fall_2 = 0;
float tick_diff;
uint8_t cycle = 0;
// uint8_t flag = false;
// uint8_t crossing = 0;

void setup()
{
  cli();
  
  // Reset Arduino IDE Timer1 presets.
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  Serial.begin(9600);

  DDRB |= (1 << PB2) | (1 << PB4);
  PORTB &= ~(1 << PB4); // Initially set PB4 low.

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
  TIMSK1 |= (1 << TOIE1);

  // When AIN1 is LOW, comparator output is HIGH. Therefore, the edge detection of input capture should be flipped.
  // Analog Comparator configuration.
  ADCSRB &= ~(1 << ACME); // AIN1 is applied to the negative input of the analog comparator.
  ACSR = 0;
  ACSR |= (1 << ACBG) | (1 << ACIC); // Bandgap reference, and input capture.
  TIFR1 |= (1 << ICF1); // Clear input capture flag that was set by changing the input capture source.

  sei();// Enable Global Interrupts
  
  
  // Timer1 start.
  // // CLK / 8 prescaler.
  // TCCR1B &= ~((1 << CS12) | (1 << CS10));
  // TCCR1B |= (1 << CS11);
  
  // CLK / 1 prescaler. No Prescaler.
  TCCR1B &= ~((1 << CS12) | (1 << CS11));
  TCCR1B |= (1 << CS10);
}

void loop()
{
  // if (cycle == 1)
  // {
  Serial.print("timestamp fall 1 = ");
  Serial.print(toggle_stamp_fall_1);

  Serial.print(", timestamp fall 2 = ");
  Serial.print(toggle_stamp_fall_2);
  
  Serial.print(", Tick diff = ");
  Serial.print(tick_diff);
  
  float period = tick_diff / 16; // Timestamp is in 62.5 ns/tick. t_fall-t_rise = 16 * T/2 => T = (t_fall - trise)/16
  float inductance = period * period / L_CONSTANT;
  Serial.print(", Inductance = ");
  Serial.print(inductance);
  Serial.println("uH");
}

// first trigger on falling-edge of comparator, then trigger on rising edge to capture one half-cycle.

ISR(TIMER1_CAPT_vect)
{
  if (cycle == 2)
  {
    toggle_stamp_fall_2 = ICR1;
    tick_diff = toggle_stamp_fall_2 - toggle_stamp_fall_1;
  }
  else if (cycle == 1)
  {
    toggle_stamp_fall_1 = ICR1;
  }

  cycle++;
}


ISR(TIMER1_OVF_vect)
{
  cycle = 0;
}
