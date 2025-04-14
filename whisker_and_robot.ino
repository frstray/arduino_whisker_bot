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
  int maxSpeed = 255;
  int baseSpeed = 200; 

// Flex Sensor
  // analog pins for reading from the flex sensors
  const byte FLEX_PIN_1 = A0;
  const byte FLEX_PIN_2 = A1;

  // variables for storing readings from each sensor in bits
  float flex_reading_1; float flex_reading_2;
  float previous_flex_1; float previous_flex_2;
  float filtered_flex_1; float filtered_flex_2;

  // variables for storing base line reading for each sensor 
  float baseline_1; float baseline_2; 

  // flex thresholds 
  // this is what we are considering to be the extreme value for a reading from the flex sensor
  // the true max value would be unreasonable to reach for practical purposes, so we are picking a max suited to the situation
  float flex_stop = 50;
 
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
  float AdjustSpeed(float flex_thresholdP, float flex_sensor_readingP, int base_speedP){
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

/*****************************************************************************************/
/* SET UP */
void setup() {
  Serial.begin(115200);
  
  // Setup Motor
    // right motor
    pinMode(ena, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
    // left motor
    pinMode(enb, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT); 

    Serial.println("Starting up...");

  // Set up the flex sensors 
    // Get Base line readings for the flex sensors
    int number_of_readings = 10;
    float sum_1 = 0; float sum_2 = 0;

  // sum readings for "number_of_readings" loops
    for(int ii = 0; ii < number_of_readings; ii++) {
      sum_1 = sum_1 + analogRead(FLEX_PIN_1);
      sum_2 = sum_2 + analogRead(FLEX_PIN_2);
    }

    // take the average of readings in order to set the baseline
    baseline_1 = sum_1/float(number_of_readings);
    baseline_2 = sum_2/float(number_of_readings);

    // set previous flex variables before entering void loop so filtering works
    previous_flex_1 = analogRead(FLEX_PIN_1) - baseline_1;
    previous_flex_2 = analogRead(FLEX_PIN_2) - baseline_2;

}

/*****************************************************************************************/
/* LOOP */
void loop() {
  // Read the flex sensors
  flex_reading_1 = analogRead(FLEX_PIN_1) - baseline_1;
  flex_reading_2 = analogRead(FLEX_PIN_2) - baseline_2; 

  // filter the data
  // We are applying a filter so we aren't picking up on vibrations 
  filtered_flex_1 = (0.90 * previous_flex_1) + (0.10 * flex_reading_1);
  filtered_flex_2 = (0.90 * previous_flex_2) + (0.10 * flex_reading_2);

  /* uncomment to print the data */
  /*
  Serial.print(flex_reading_1); Serial.print(", ");
  Serial.print(flex_reading_2); Serial.print(", "); 
  Serial.print(filtered_flex_1); Serial.print(", ");
  Serial.println(filter_flex_2);
  */


  // adjust the speed of the motors based on readings from the flex sensors 
  RightMotor(AdjustSpeed(flex_stop, filtered_flex_1, baseSpeed), 'F');
  LeftMotor(AdjustSpeed(flex_stop, filtered_flex_2, baseSpeed), 'F');
  //Serial.print(right_speed); Serial.print(", "); Serial.println(left_speed);

   
   //update values 
   previous_flex_1 = filtered_flex_1;
   previous_flex_2 = filtered_flex_2;
   
}
