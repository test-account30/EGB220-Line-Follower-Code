#include <avr/interrupt.h>

//######################################
// Start State Machine Variables
//######################################

typedef enum {
  stopped,         // Robot stopped
  straight,        // Robot driving straight
  turning,         // Robot Turning
  switch_path,     // Robot navigating which split in path to take
  intersection     // Robot ignoring intersection
} robot_state;

robot_state state = straight;

typedef enum {
  red_white,
  white,
  red,
  green_white,
  green,
  black
} sensor_colour;

sensor_colour colour = black;

//######################################
// Start IR Sensor Functions:
//######################################

void ir_init(void);
float ir_get_angle(void);
float ir_get_setpoint(uint8_t side_bias);  // Adjusted to remove unused ir_get_count function declaration
void left_sensor_init(void);

//######################################
// Start LED functions
//######################################

void LED_init(void);
void led_set_state(uint8_t led_no, uint8_t state);

//######################################
// Start PID functions:
//######################################

typedef struct {
  float kp;             // Proportional gain
  float ki;             // Integral gain
  float kd;             // Derivative gain
  float previous_error; // Previous error for derivative calculation
  float integral;       // Integral term for integral action
  uint16_t start;       // Start time for calculating dt
  float min_i;          // Minimum value for integral wind-up prevention
  float max_i;          // Maximum value for integral wind-up prevention
} PID;

void setupPID(PID *pid, float kp, float ki, float kd, float min_i, float max_i);
float calculatePID(PID *pid, float setpoint, float measured_value, int16_t max, int16_t min);

//######################################
// Start PWM functions:
//######################################

void pwm_clear();
void pwm_init();
void pwm_speed(uint8_t speed1, uint8_t dir1, uint8_t speed2, uint8_t dir2);

//######################################
// Start ADC functions:
//######################################

void ADC_init(void);
uint8_t read_ADC(uint8_t mux_no);

void timer_init(void);

//********** MAIN SCRIPT **********

static PID steering;

void setup() {
  ADC_init();         // Initialize ADC
  timer_init();       // Initialize Timer
  sei();              // Enable global interrupts
  Serial.begin(9600); // Initialize serial communication
  setupPID(&steering, 5, 10, 6, -10, 10); // Initialize PID controller
  pwm_clear();        // Clear PWM
  pwm_init();         // Initialize PWM
  ir_init();          // Initialize IR sensors
  LED_init();         // Initialize LEDs
  left_sensor_init(); // Initialize left sensor
  DDRD &= ~(1<<3);    // Enable bumper input
}

float l_speed = 20;          // Left motor speed
float r_speed = 20;          // Right motor speed
float nominal_speed = 80;    // Nominal motor speed
float steering_command = 0;  // Steering command
uint8_t intersection_marker_count = 0;  // Count for intersection markers
float setpoint = 0;          // PID setpoint
uint8_t intersection_dir = 0; // Direction of intersection
uint16_t count4 = 0;         // Counter variable
uint8_t speed_state = 0;     // Speed state variable
uint16_t slow_zone_delay = 0; // Delay variable for slow zones
uint8_t side_marker_count;   // Count for side markers
uint8_t side_marker_trig = 0; // Trigger for side markers
uint16_t max_count_val_timer = 0; // Timer for maximum count value
uint16_t turn_off_turn_count = 0; // Counter for turning off turn
uint8_t enter_turn_light_counter = 0; // Counter for entering turn light
uint8_t is_intersection = 0; // Intersection flag
uint8_t is_white_line = 0;   // White line flag
uint16_t is_white_line_counter = 0; // Counter for white line
uint16_t stopping_timer = 0; // Timer for stopping
uint8_t stop_flipflop = 0;   // Flip-flop for stopping
uint8_t have_adjusted_speed = 0; // Adjusted speed flag
uint16_t slow_zone_delay_value = 0; // Delay value for slow zone

void loop() {
  float robot_position = -ir_get_angle();  // Get robot's position from IR sensor
  
  // PID control based on robot's position
  if (abs(robot_position) != 100){ 
    steering_command = calculatePID(&steering, setpoint * abs(setpoint), robot_position * abs(robot_position), 128, -128);
  }
  
  // Adjust motor speeds based on steering command
  l_speed = r_speed = nominal_speed;
  if (steering_command >= 0){
    r_speed = nominal_speed;
    l_speed = nominal_speed * (1 - abs(steering_command) / 128);
  } else {
    l_speed = nominal_speed;
    r_speed = nominal_speed * (1 - abs(steering_command) / 128);
  }

  // Read ADC values and output to serial
  Serial.println(read_ADC(0));

  // State machine based on color sensor readings
  switch (colour) {
    case red_white:
      Serial.println("Redwhite");
      if (read_ADC(0) < 136){ 
        if (read_ADC(0) < 110){
          colour = white;
        }
        count4++;
      } else {
        count4 = 0;
      }
      if (count4 > 50){
        colour = red;
        count4 = 0;
        intersection_dir = !intersection_dir;
      }
      Serial.println(count4);
      break;
    case red:
      Serial.println("Red");
      is_intersection = 1;
      break;
    case white:
      if (max_count_val < 10)
        is_white_line = 1;
      else {
        colour = black;
        max_count_val = 0;
      }
      Serial.println("white");
      break;
    case green:
      break;
    case green_white:
      break; 
    case black:
      if (read_ADC(1) < 70 && (slow_zone_delay == 0)){ 
        colour = red_white;
      } 
      break;
    default:
      colour = black;
  }

  // Update max count value from IR sensor
  if (max_count_val < ir_get_count()){
    max_count_val = ir_get_count();
  }

  // Side marker logic
  if (!(PINC & (1 << 6))){
    side_marker_count++;
  } else{
    side_marker_count = 0;
  }

  if (side_marker_trig){
    side_marker_on_count++;
  }

  if(side_marker_on_count > 800){
    side_marker_trig = 0;
    side_marker_on_count = 0;
  }

  if ((side_marker_count > 20)){
    max_count_val = 0;
    side_marker_count = 0;
    side_marker_trig = 1;
    max_count_val_timer = 0;
  }

  if (max_count_val >= 10){
    max_count_val_timer++;
  }

  if (max_count_val_timer > 500){
    max_count_val_timer = 0;
    max_count_val = 0;
  }

  // State machine logic
  switch (state){
    case straight:
      if (is_white_line & (max_count_val < 10)){
        is_white_line_counter++;
      } else {
        is_white_line = 0;
        is_white_line_counter = 0;
      }
      if (is_white_line_counter > 800){
        is_white_line = 0;
        colour = black;
        is_white_line_counter = 0;
        if (stop_flipflop)
          state = stopped;
        stop_flipflop = !stop_flipflop;
      }

      led_set_state(2, 1);
      led_set_state(1, 0);
      if (side_marker_trig){
        if (abs(steering_command) > 15){
          enter_turn_light_counter++;
        } else {
          enter_turn_light_counter = 0;
        }
      }

      if (abs(steering_command) > 25){
        enter_turn_light_counter++;
      } else {
        enter_turn_light_counter = 0;
      }

      if ((enter_turn_light_counter > 20)){
        enter_turn_light_counter = 0;
        side_marker_trig = 0;
        state = turning;
        side_marker_count = 0; 
      }
      break;
    case stopped:
      led_set_state(0, 1);
      led_set_state(2, 0);
      led_set_state(1, 0);
      stopping_timer++;
      l_speed = 0;
      r_speed = 0;
      if (stopping_timer > 5000){
        stopping_timer = 0;
        led_set_state(0, 0);
        led_set_state(2, 1);
        state = straight;
      }
      break;
    case turning:
      is_white_line = 0;
      is_white_line_counter = 0;
      led_set_state(1, 1);
      led_set_state(2, 0);
      if (abs(steering_command) < 10){
        turn_off_turn_count++;
      } else {
        turn_off_turn_count = 0;
      }
      if ((turn_off_turn_turn_count > 15 + is_intersection * 15)){
        turn_off_turn_count = 0;
        led_set_state(1, 0);
        state = straight;
      }
      break;
    default:
      state = straight;
  }

  // Intersection handling
  if (is_intersection){
    Serial.println("Intersection");
    if (!speed_state){
      nominal_speed /= 1.5; // Reduce speed for intersection
      speed_state = 1;
    }
    if (((PIND & (1 << 3))) && (have_adjusted_speed < 250)){
      have_adjusted_speed++;
    } 
    if (have_adjusted_speed > 200 && !(have_adjusted_speed == 255)){
      nominal_speed /= 1.8; // Further reduce speed after some time
      have_adjusted_speed = 255;
      slow_zone_delay -= 2000; // Adjust slow zone delay
    }
    slow_zone_delay++;
    if (slow_zone_delay < 2000){ // Delay until intersection handling ends
      setpoint = ir_get_setpoint(intersection_dir);
    } else {
      setpoint = 0;
    }
    if (intersection_dir){
      slow_zone_delay_value = 14000; // Set delay value for intersection direction
    } else {
      slow_zone_delay_value = 6000;
    }

    if (slow_zone_delay > slow_zone_delay_value){
      slow_zone_delay = 0;
      nominal_speed *= 1.5; // Restore nominal speed
      if (have_adjusted_speed == 255){
        nominal_speed *= 1.8; // Restore adjusted speed
      }
      have_adjusted_speed = 0;
      speed_state = 0;
      is_intersection = 0;
      colour = black;
      state = straight;
      led_set_state(0, 0);
    }
  } else {
    setpoint = 0;
  }

  // Determine motor direction based on speed
  uint8_t dir_l = (l_speed >= 0) ? 0 : 1;
  uint8_t dir_r = (r_speed >= 0) ? 0 : 1;
  pwm_speed(l_speed, dir_l, r_speed, dir_r); // Set motor speeds
}

