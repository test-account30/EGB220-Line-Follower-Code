


//######################################
// Start PWM functions:
//######################################

void pwm_clear(void){
    cli(); //THis disables interupts
    TCCR0A = 0;  //Clears Timer 0 control register A
    TCCR0B = 0;  //Clears Timer 0 control register B
    OCR0A = 0;   //Sets the compare value A to 0
    OCR0B = 0;   //Sets the compare value B to 0
    sei();
}

void pwm_init(void){
  cli();
  TCCR0A |= (1<<COM0B1) | (1<<COM0A1) | (1<<WGM00) | (1<<WGM01);    //sets it to FAST PWM, and COM0B1/COM0A1 lets it controlthe OCOA/B registers
  TCCR0B |= (1<<CS01)|(1<CS00);    //sets the prescaler to 1024 and turns on the clock
  OCR0A = 0x0;  //Sets the compare value A to 0   (Output off Cleared)
  OCR0B = 0x0;  //Sets the compare value B to 0   (Output off Cleared)

  //Motor 1:
  DDRB |= (1<<0);  //Turns the phase port to output, which decides the rotational direction for motor 2
  PORTB &= ~(1<<0); //default value (off)
  DDRB |= (1<<7);  //Enables output for motor 1

  //Motor 2:
  DDRE |=(1<<6);  //Turns the phase port to output, which decides the rotational direction for motor 1 
  PORTE &= ~(1<<6); //default value (off)
  DDRD |=(1<<0);  //Enables output for motor 2
  sei();
}

void pwm_stop(void){
    cli();
    OCR0A = 0x0;   //Turns the A comparison Value to the min, which turns the output off 
    OCR0B = 0x0;   //Turns the B comparison Value to the min, which turns the output off 
    sei();
}



void pwm_speed(uint8_t speed1, uint8_t dir1, uint8_t speed2, uint8_t dir2){
  //THis is a vairable speed equation. It accepts an 8 bit number, max 255, min 0. A max number will be a 100% duty cycle.
  // A min will be a 0%, where as 126, or 0x80 is 50% duty cycle. The higher the number the greater the percentage. 
  cli();
  if (dir1){
    PORTB |= (1<<0);
  } else{
    PORTB &= ~(1<<0);
  }
  if (dir2){
    PORTE |= (1<<6);
  } else{
    PORTE &= ~(1<<6);
  }
  OCR0A = speed1;
  OCR0B = speed2;
  sei();
}

