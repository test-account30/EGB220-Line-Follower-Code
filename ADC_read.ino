//######################################
// Start Timer/Counter functions:
//######################################


void ADC_init(void){
  ADCSRA |= (1<<7)|(1<<2)|(1<<1)|1; // setup ADC in signal conversion mode
  ADCSRB = 0;
}

uint8_t read_ADC(uint8_t mux_no){
  switch (mux_no){
    case 0:
      ADMUX |= (1<<6)|(1<<5);
      ADMUX &= ~(1<<2);
      break;
    case 1:
      ADMUX |= (1<<6)|(1<<5)|(1<<2);
      break;
  }
  ADCSRA |= (1 << 6);
  while (ADCSRA & (1<<6)); // blocking function shh don't tell anyone
  return ADCH;
} 