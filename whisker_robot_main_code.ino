/*
 * Forest Ray
 * May 14, 2025
 * "Whisker" Robot 
 * 
 * The following code is for a 4-wheeled robot with DC motors. It takes input from flex sensors pointing to the left and right at the front of the robot.
 * It is meant for navigating narrow tunnels where the flex sensors are touching the walls more often than not. 
 */

// globals
// Motor
  // Motor 1 
  const byte ena = 6;
  const byte in1 = 7;
  const byte in2 = 5;

  // Motor 2 
  const byte enb = 8;
  const byte in3 = 9;
  const byte in4 = 10; 

  // Default Motor Speeds
  int maxSpeed  = 255;
  int baseSpeed = 180;  

  float adjusted_speed_top_right; float adjusted_speed_bottom_right;
  float adjusted_speed_top_left;  float adjusted_speed_bottom_left;

// Flex Sensor
  /*To do: maybe change the flex pin names to reflect location on robot
  * add variables for the other four flex sensors
  * change the name of the threshold variable
  */ 
  
  // analog pins for reading from the flex sensors
  const byte FLEX_PIN_TOP_RIGHT    = A0;
  const byte FLEX_PIN_BOTTOM_RIGHT = A1;
  const byte FLEX_PIN_TOP_LEFT     = A2;
  const byte FLEX_PIN_BOTTOM_LEFT  = A3;
  const byte FLEX_PIN_HEAD_RIGHT   = A4;
  const byte FLEX_PIN_HEAD_LEFT    = A5;

  // variables for storing readings from each sensor in bits
  float flex_reading_top_right;  float flex_reading_bottom_right;  float flex_reading_top_left;  float flex_reading_bottom_left;  float flex_reading_head_right;  float flex_reading_head_left;
  float previous_flex_top_right; float previous_flex_bottom_right; float previous_flex_top_left; float previous_flex_bottom_left; float previous_flex_head_right; float previous_flex_head_left; 
  float filtered_flex_top_right; float filtered_flex_bottom_right; float filtered_flex_top_left; float filtered_flex_bottom_left; float filtered_flex_head_right; float filtered_flex_head_left;

  // variables for storing base line reading for each sensor 
  float baseline_top_right; float baseline_bottom_right; float baseline_top_left; float baseline_bottom_left; float baseline_head_right; float baseline_head_left;  

  // flex thresholds 
  // this is what we are considering to be the extreme value for a reading from the flex sensor
  // the true max value would be unreasonable to reach for practical purposes, so we are picking a max suited to the situation
  // these values vary for each flex sensor
  float flex_high_top_left     = 30;
  float flex_high_bottom_left  = 40;
  float flex_high_top_right    = 30;
  float flex_high_bottom_right = 30;
  float flex_high_head_right   = 20;
  float flex_high_head_left    = 20;
 
/* Functions */

/* Handle Motor Control */
  // Right Motor Control (ena)
  void RightMotor(int SpeedP, char directionP){
    /* Arguments: 
     * SpeedP is the speed parameter that will dictate power given to the motors
     * directionP will tell the motors whether they need to go forward or backward
     *  - This can be F for forward or B for backward
    */

     if (directionP == 'F'){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(ena, SpeedP);
     } else if (directionP == 'B') {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(ena, SpeedP);
     }
  }
  // Left Motor Control (enb)
  void LeftMotor(int SpeedP, char directionP){
    /* Arguments: 
     * SpeedP is the speed parameter that will dictate power given to the motors
     * directionP will tell the motors whether they need to go forward or backward
     *  - This can be F for forward or B for backward
    */

    if (directionP == 'F'){
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enb, SpeedP);
    } else if (directionP == 'B') {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enb, SpeedP);
    }
  }
  // whisker speed adjustment
  int DefaultSpeedAdjust(float flex_thresholdP, float flex_sensor_readingP, int base_speedP){
    // if pressure on left, we want to slow down right (turn right)
    // if pressure on right we want to slow down left (turn left) 
    // so we want to adjust speed by providing a lower speed based on input
    int new_speed; // variable storing the new speed
    
    // if sensor reading negative
    if (flex_sensor_readingP < 0){
      // if the sensor is reading less than 0, this means that the flex sensor is bending the opposite way to what we expect
      // while we would typically slow down the motors in response to a change, in this case we want the motors to speed up
      // so that the robot turns in the opposite direction of what we would tell it too if the flex sensor was bending
      // in the other direction
      new_speed = (((base_speedP/flex_thresholdP) * (flex_thresholdP-flex_sensor_readingP))/flex_thresholdP) + base_speedP;
    }

    // if sensor reading greater than max threshold
    else if (flex_sensor_readingP > flex_thresholdP){
      // if the flex sensor reading is over our maximum threshold, stop that motor 
      new_speed = 0;
    }

    // if sensor reading in typical range 
    else if(flex_sensor_readingP <= flex_thresholdP & flex_sensor_readingP >= 0){
      // when more flex is applied to a whisker on one side, we want to turn away from that direction
      // we accomplish this by slowing down the motor on the opposite side 
      new_speed = (base_speedP/flex_thresholdP) * (flex_thresholdP - flex_sensor_readingP);
    }

    return new_speed;
  }

  int WeightedSpeedAdjust(float flex_thresholdP, float flex_sensor_readingP, int base_speedP){
    // if pressure on left, we want to slow down right (turn right)
    // if pressure on right we want to slow down left (turn left) 
    // so we want to adjust speed by providing a lower speed based on input
    int new_speed; // variable storing the new speed
    
    // if sensor reading negative
    if (flex_sensor_readingP < 0){
      // if the sensor is reading less than 0, this means that the flex sensor is bending the opposite way to what we expect
      // while we would typically slow down the motors in response to a change, in this case we want the motors to speed up
      // so that the robot turns in the opposite direction of what we would tell it too if the flex sensor was bending
      // in the other direction
      new_speed = (((base_speedP/flex_thresholdP) * (flex_thresholdP-flex_sensor_readingP-(0.10*flex_thresholdP)))/flex_thresholdP) + base_speedP;
    }

    // if sensor reading greater than max threshold
    else if (flex_sensor_readingP > flex_thresholdP){
      // if the flex sensor reading is over our maximum threshold, stop that motor 
      new_speed = 0;
    }

    // if sensor reading in typical range 
    else if(flex_sensor_readingP <= flex_thresholdP & flex_sensor_readingP >= 0){
      // when more flex is applied to a whisker on one side, we want to turn away from that direction
      // we accomplish this by slowing down the motor on the opposite side 
      new_speed = (base_speedP/flex_thresholdP) * (flex_thresholdP - flex_sensor_readingP - (0.10*flex_thresholdP));
    }

    return new_speed;
  }

  char GetSensorState(float filtered_reading_top, float filtered_reading_bottom, float max_top, float max_bottom, float dead_zone_threshold){
    if( (filtered_reading_top > (-1*dead_zone_threshold)) && (filtered_reading_top < dead_zone_threshold) ){
      if( (filtered_reading_bottom > dead_zone_threshold) && (filtered_reading_bottom < (0.267*max_bottom)) ){
        return 'B';
      } else if ( (filtered_reading_bottom > (-1*dead_zone_threshold)) && (filtered_reading_bottom < dead_zone_threshold) ){
        return 'A';       
      } else { return 'Z'; } // error value
    } else if (filtered_reading_top > (0.5*max_top)) {
      if (filtered_reading_bottom > (0.833*max_bottom)) {
        return 'D';
      } else if (filtered_reading_bottom > (0.25*max_bottom)){
        return 'C';
      } else { return 'Z'; } // error value
    } else { return 'Z'; } // error value
  }

  int GetRobotState(float top_left, float bottom_left, float top_right, float bottom_right, float max_top_left, float max_bottom_left, float max_top_right, float max_bottom_right, float dead_zone_threshold){
    // get sensor state
    char left_state  = GetSensorState(top_left,  bottom_left,  max_top_left,  max_bottom_left, dead_zone_threshold);
    char right_state = GetSensorState(top_right, bottom_right, max_top_right, max_bottom_right, dead_zone_threshold);

    int robot_state = 2; 

    switch (left_state){
      // left A
      case 'A':
        switch (right_state){
          case 'A':
            robot_state = 1;
            break;
          case 'B':
            robot_state = 2;
            break;
          case 'C':
            robot_state = 3;
            break;
          case 'D':
            robot_state = 4;
            break;
          default:
            //Serial.println("Robot State Error");          
            robot_state = 2;
            break;
        } 
        break;
      // left B
      case 'B':
        switch (right_state){
          case 'A':
            robot_state = 2;
            break;
          case 'B':
            robot_state = 2;
            break;
          case 'C':
            robot_state = 3;
            break;
          case 'D':
            robot_state = 4;
            break;
          default:
            //Serial.println("Robot State Error");
            robot_state = 2;
            break;
          } 
          break;
      
      // left C
      case 'C':
        switch(right_state){
          case 'A':
            robot_state = 3;
            break;
          case 'B':
            robot_state = 3;
            break;
          case 'C':
            robot_state = 2;
            break;
          case 'D':
            robot_state = 5;
            break;

          default:
            //Serial.println("Robot State Error");
            robot_state = 2;
            break;
        }
        break;

      // left D
      case 'D':
        switch (right_state) {
          case 'A':
            robot_state = 4;
            break;
          case 'B':
            robot_state = 4;
            break;
          case 'C':
            robot_state = 5;
            break;
          case 'D':
            robot_state = 5; 
            break;
            
          default:
            //Serial.println("Robot State Error");
            robot_state = 2;
            break;
        }
        break;

      default:
        //Serial.println("Robot State Error");
        robot_state = 2;
        break;
     } 

     return robot_state;
  }

  void AdjustMotorSpeed(float top_left, float bottom_left, float top_right, float bottom_right, float max_top_left, float max_bottom_left, float max_top_right, float max_bottom_right, float dead_zone_threshold, int baseSpeed){
    int robo_state = GetRobotState(top_left, bottom_left, top_right, bottom_right, max_top_left, max_bottom_left, max_top_right, max_bottom_right, dead_zone_threshold);
    int right_motor_speed; int left_motor_speed;
    //Serial.println(robo_state);
    
    switch (robo_state){
      case 1:
        LeftMotor(baseSpeed, 'F');
        RightMotor(baseSpeed, 'F');
        break;
      case 2:
        if ( ((top_right > (-1*dead_zone_threshold)) && (top_right < dead_zone_threshold)) ){
          right_motor_speed = DefaultSpeedAdjust(max_bottom_right, bottom_right, baseSpeed);
        } else {
          right_motor_speed = (DefaultSpeedAdjust(max_top_right, top_right, baseSpeed) + DefaultSpeedAdjust(max_bottom_right, bottom_right, baseSpeed))/2;
        }

        if ( ((top_left > (-1*dead_zone_threshold)) && (top_left < dead_zone_threshold)) ){
          left_motor_speed = DefaultSpeedAdjust(max_bottom_left, bottom_left, baseSpeed);
        } else {
          left_motor_speed = (DefaultSpeedAdjust(max_top_left, top_left, baseSpeed) + DefaultSpeedAdjust(max_bottom_left, bottom_left, baseSpeed))/2;
        }

        //Serial.println(left_motor_speed);
        //Serial.println(right_motor_speed);
        RightMotor(right_motor_speed, 'F');
        LeftMotor(left_motor_speed, 'F');
        break;
      
      case 3:
        if (bottom_right > bottom_left){
          // weighted on left, normal on right, turn more left
          if ( ((top_right > (-1*dead_zone_threshold)) && (top_right < dead_zone_threshold)) ){
            right_motor_speed = DefaultSpeedAdjust(max_bottom_right, max_bottom_right, baseSpeed);
          } else {
            right_motor_speed = (DefaultSpeedAdjust(max_top_right, top_right, baseSpeed) + DefaultSpeedAdjust(max_bottom_right, bottom_right, baseSpeed))/2;
          }

          if ( ((top_left > (-1*dead_zone_threshold)) && (top_left < dead_zone_threshold)) ){
            left_motor_speed = WeightedSpeedAdjust(max_bottom_left, bottom_left, baseSpeed);
          } else {
            left_motor_speed = (WeightedSpeedAdjust(max_top_left, top_left, baseSpeed) + WeightedSpeedAdjust(max_bottom_left, bottom_left, baseSpeed))/2;
          }
         } 
         
        else if (bottom_right < bottom_left) {
          // weighted on right, normal on left, turn more right
          if ( ((top_right > (-1*dead_zone_threshold)) && (top_right < dead_zone_threshold)) ){
            right_motor_speed = WeightedSpeedAdjust(max_bottom_right, max_bottom_right, baseSpeed);
          } else {
            right_motor_speed = (WeightedSpeedAdjust(max_top_right, top_right, baseSpeed) + WeightedSpeedAdjust(max_bottom_right, bottom_right, baseSpeed))/2;
          }

          if ( ((top_left > (-1*dead_zone_threshold)) && (top_left < dead_zone_threshold)) ){
            left_motor_speed = DefaultSpeedAdjust(max_bottom_left, bottom_left, baseSpeed);
          } else {
            left_motor_speed = (DefaultSpeedAdjust(max_top_left, top_left, baseSpeed) + DefaultSpeedAdjust(max_bottom_left, bottom_left, baseSpeed))/2;
          }
        }
  
        RightMotor(right_motor_speed, 'F');
        LeftMotor(left_motor_speed, 'F');
        break;

      case 4:
        if(bottom_right > bottom_left){
          // turn left
          RightMotor(200, 'F'); LeftMotor(0, 'F'); 
          delay(150);

          RightMotor(0, 'F'); LeftMotor(0, 'F');
          delay(100);
    
          // back up slightly to reduce bend
          RightMotor(baseSpeed, 'B'); LeftMotor(baseSpeed, 'B');
          delay(150);

          RightMotor(0, 'F'); LeftMotor(0, 'F');
        } else if(bottom_left > bottom_right){
          // turn right
          RightMotor(0, 'F'); LeftMotor(200, 'F');
          delay(150);

          RightMotor(0, 'F'); LeftMotor(0, 'F');
          delay(100);
    
          // back up slightly to reduce bend
          RightMotor(baseSpeed, 'B'); LeftMotor(baseSpeed, 'B');
          delay(150);
    
          RightMotor(0, 'F'); LeftMotor(0, 'F');
        }
        break;

      case 5:
        RightMotor(0, 'F'); LeftMotor(0, 'F');
        break;
      default:
        RightMotor(0, 'F'); LeftMotor(0, 'F');
        break;
    }
  }

/*****************************************************************************************/
/* SET UP */
void setup() {
  Serial.begin(9600);
  
  // Setup Motor
    // right motor
    pinMode(ena, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
    // left motor
    pinMode(enb, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT); 

    Serial.println("Starting up...");

  // Set up the flex sensors 
    // Get Base line readings for the flex sensors
    int number_of_readings = 50;
    
    float sum_top_right  = 0; float sum_bottom_right = 0; 
    float sum_top_left   = 0; float sum_bottom_left  = 0;
    float sum_head_right = 0; float sum_head_left    = 0;

  // sum readings for "number_of_readings" loops
    for(int ii = 0; ii < number_of_readings; ii++) {
      sum_top_right    = sum_top_right    + analogRead(FLEX_PIN_TOP_RIGHT);
      sum_bottom_right = sum_bottom_right + analogRead(FLEX_PIN_BOTTOM_RIGHT);
      sum_top_left     = sum_top_left     + analogRead(FLEX_PIN_TOP_LEFT);
      sum_bottom_left  = sum_bottom_left  + analogRead(FLEX_PIN_BOTTOM_LEFT);
      sum_head_right   = sum_head_right   + analogRead(FLEX_PIN_HEAD_RIGHT);
      sum_head_left    = sum_head_left    + analogRead(FLEX_PIN_HEAD_LEFT);
    }

    // take the average of readings in order to set the baseline
    baseline_top_right    = sum_top_right/float(number_of_readings);
    baseline_bottom_right = sum_bottom_right/float(number_of_readings);
    baseline_top_left     = sum_top_left/float(number_of_readings);
    baseline_bottom_left  = sum_bottom_left/float(number_of_readings);
    baseline_head_right   = sum_head_right/float(number_of_readings);
    baseline_head_left    = sum_head_left/float(number_of_readings);

    // set previous flex variables before entering void loop so filtering works
    previous_flex_top_right    = analogRead(FLEX_PIN_TOP_RIGHT)    - baseline_top_right;
    previous_flex_bottom_right = analogRead(FLEX_PIN_BOTTOM_RIGHT) - baseline_bottom_right;
    previous_flex_top_left     = analogRead(FLEX_PIN_TOP_LEFT)     -  baseline_top_left;
    previous_flex_bottom_left  = analogRead(FLEX_PIN_BOTTOM_LEFT)  - baseline_bottom_left;
    previous_flex_head_right   = analogRead(FLEX_PIN_HEAD_RIGHT)   - baseline_head_right;
    previous_flex_head_left    = analogRead(FLEX_PIN_HEAD_LEFT)    - baseline_head_left;

}

/*****************************************************************************************/
/* LOOP */
void loop() {
  // Read the flex sensors
  flex_reading_top_right    = analogRead(FLEX_PIN_TOP_RIGHT)    - baseline_top_right;
  flex_reading_bottom_right = analogRead(FLEX_PIN_BOTTOM_RIGHT) - baseline_bottom_right;
  flex_reading_top_left     = analogRead(FLEX_PIN_TOP_LEFT)     - baseline_top_left;
  flex_reading_bottom_left  = analogRead(FLEX_PIN_BOTTOM_LEFT)  - baseline_bottom_left; 
  flex_reading_head_right   = analogRead(FLEX_PIN_HEAD_RIGHT)   - baseline_head_right;
  flex_reading_head_left    = analogRead(FLEX_PIN_HEAD_LEFT)    - baseline_head_left;

  // filter the data
  // We are applying a filter so we aren't picking up on vibrations 
  filtered_flex_top_right    = (0.80 * previous_flex_top_right)    + (0.20 * flex_reading_top_right);
  filtered_flex_bottom_right = (0.80 * previous_flex_bottom_right) + (0.20 * flex_reading_bottom_right);
  filtered_flex_top_left     = (0.80 * previous_flex_top_left)     + (0.20 * flex_reading_top_left);
  filtered_flex_bottom_left  = (0.80 * previous_flex_bottom_left)  + (0.20 * flex_reading_bottom_left);
  filtered_flex_head_right   = (0.80 * previous_flex_head_right)   + (0.20 * flex_reading_head_right);
  filtered_flex_head_left    = (0.80 * previous_flex_head_left)    + (0.20 * flex_reading_head_left);

  /* uncomment to print the data */
  
  Serial.print(filtered_flex_top_right); Serial.print(", ");
  Serial.print(filtered_flex_bottom_right); Serial.print(", ");
  Serial.print(filtered_flex_top_left); Serial.print(", ");
  Serial.println(filtered_flex_bottom_left);

  /* MOTOR CONTROL
   * Stop if the flex sensors on the head or sides bend a certain distance (walls are too close)
   * Adjust the speeds based on sensor flex (average from top and bottom sensors for each side)
   */
   
  // stop conditions
  if ( ((filtered_flex_top_right > flex_high_top_right) || (filtered_flex_bottom_right > flex_high_bottom_right)) && ((filtered_flex_top_left > flex_high_top_left) || (filtered_flex_bottom_left > flex_high_bottom_left)) ){
    RightMotor(0, 'F');
    LeftMotor(0, 'F');
  } else {
    AdjustMotorSpeed(filtered_flex_top_left, filtered_flex_bottom_left, filtered_flex_top_right, filtered_flex_bottom_right, flex_high_top_left, flex_high_bottom_left, flex_high_top_right, flex_high_bottom_right, 4, baseSpeed);
  }

  // void AdjustMotorSpeed(float top_left, float bottom_left, float top_right, float bottom_right, float max_top_left, float max_bottom_left, float max_top_right, float max_bottom_right, float dead_zone_threshold, int baseSpeed)
  // need a nice clean data print here for graphing 
   
   //update values 
   previous_flex_top_right    = filtered_flex_top_right;
   previous_flex_bottom_right = filtered_flex_bottom_right;
   previous_flex_top_left     = filtered_flex_top_left;
   previous_flex_bottom_left  = filtered_flex_bottom_left;
   previous_flex_head_right   = filtered_flex_head_right;
   previous_flex_head_left    = filtered_flex_head_left;
   
}
