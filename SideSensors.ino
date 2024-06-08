//######################################
// Side Sensors 
//######################################


void left_sensor_init(void){
  DDRC &= ~(1<<6);  //turns on in A
}


