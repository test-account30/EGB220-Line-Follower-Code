//######################################
// Start Timer/Counter functions:
//######################################


void timer_init(void){
  // Initialize Timer1 with prescaler 64
  TCCR1A = 0b00000000;
  TCCR1B |= (1 << CS11) | (1 << CS10);
  TCNT1 = 0;
  // Set Timer1 initial value
}