void LED_init(void){
  DDRC |= (1<<7);  //Red LED
  DDRB |= (1<<5);  //Yellow LED
  DDRB |= (1<<6);  //Green LED
  PORTC &= ~(1<<7);
  PORTB &= ~(1<<5);
  PORTB &= ~(1<<6);
} 

void led_set_state (uint8_t led_no, uint8_t state){
  if (led_no == 0){
    if (state){
      PORTC |= (1<<7);
    } else{
      PORTC &= ~(1<<7);
    }
  } else if (led_no == 1){
    if (state){
      PORTB |= (1<<5);
    } else{
      PORTB &= ~(1<<5);
    }
  } else if (led_no == 2){
    if (state){
      PORTB |= (1<<6);
    } else {
      PORTB &= ~(1<<6);
    }
  }
} 