#include <SPI.h>  // include the SPI library:

float get_angle(uint16_t chaos, uint8_t is_count){
  float angle = 0;
  uint8_t count = 0;
  float adjustments[] = {-6.8, -11, 6.8, 11, -13.5, -4.6, 4.6, 13.5, -15.5, 2.3, -2.3, 15.5, -9, 0, 9};
  for (int i = 0; i < 15; i++) {
    if ((chaos & (1 << i)) != 0) {
      angle += adjustments[i];
      count++;
    }
  }
  if (is_count) return count;
  if (count != 0) return angle/count;
  return 100;
}


float get_setpoint(uint16_t raw_input, uint8_t side_bias){
  float max_value = 0;
  float adjustments[] = {-6.8, -11, 6.8, 11, -13.5, -4.6, 4.6, 13.5, -15.5, 2.3, -2.3, 15.5, -9, 0, 9};
  for (int i = 0; i < 15; i++) {
    if ((raw_input & (1 << i)) != 0) {
      if (side_bias){
        if (adjustments[i] > 0 & (abs(adjustments[i]) > abs(max_value))){
          max_value = adjustments[i]-2.3;
        }
      } else {
        if (adjustments[i] < 0 & (abs(adjustments[i]) > abs(max_value))){
          max_value = adjustments[i]+2.3;
        }
      }    
    }
  }
  return max_value;
}



void ir_init(void) {
  SPI.begin();
  DDRF |= (1 << DDF5);      // Set Load Pin as output
  PORTF &= ~(1 << PF5);     // Clear Load Pin initially
}

float ir_get_angle(void) {
  PORTF |= (1 << PF5);      // Set Load Pin to hold current state
  uint16_t data_recv = ~SPI.transfer16(0);
  data_recv &= 0x7FFF;
  PORTF &= ~(1 << PF5);     // Clear Load Pin to allow new data to be read
  return get_angle(data_recv, 0);
}

uint8_t ir_get_count(void) {
  PORTF |= (1 << PF5);      // Set Load Pin to hold current state
  uint16_t data_recv = ~SPI.transfer16(0);
  data_recv &= 0x7FFF;
  PORTF &= ~(1 << PF5);     // Clear Load Pin to allow new data to be read
  return get_angle(data_recv, 1);
}

float ir_get_setpoint(uint8_t side_bias) {
  PORTF |= (1 << PF5);      // Set Load Pin to hold current state
  uint16_t data_recv = ~SPI.transfer16(0);
  data_recv &= 0x7FFF; // remove the unused bit
  PORTF &= ~(1 << PF5);     // Clear Load Pin to allow new data to be read
  return get_setpoint(data_recv, side_bias);
}

float ir_get_bits(uint8_t side_bias) {
  PORTF |= (1 << PF5);      // Set Load Pin to hold current state
  uint16_t data_recv = ~SPI.transfer16(0);
  data_recv &= 0x7FFF; // remove the unused bit
  PORTF &= ~(1 << PF5);     // Clear Load Pin to allow new data to be read
  return data_recv;
}
