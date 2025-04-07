/*
 * Forest Ray
 * April 7, 2025
 * Flex Sensors
 * 
 * Get readings from two flex sensors and filter them so the data is less sensitive to vibrations 
 */

// Globals 

// analog pins for reading from the flex sensors 
const byte FLEX_PIN_1 = A0;
const byte FLEX_PIN_2 = A1;

// variables for storing readings from each sensor in bits
float flex_reading_1;  float flex_reading_2;
float previous_flex_1; float previous_flex_2;
float filtered_flex_1; float filtered_flex_2;

// variables for storing base line reading for each sensor 
float baseline_1; float baseline_2;

void setup() {
  Serial.begin(9600); // set baud rate

  //Serial.println("Starting up...");

  // Get Base Line readings
  int number_of_readings = 10;
  float sum_1 = 0; float sum_2 = 0;
  
  for(int ii = 0; ii < number_of_readings; ii++){
    sum_1 = sum_1 + analogRead(FLEX_PIN_1);
    sum_2 = sum_2 + analogRead(FLEX_PIN_2);
  }

  baseline_1 = sum_1/float(number_of_readings);
  baseline_2 = sum_2/float(number_of_readings);

  previous_flex_1 = analogRead(FLEX_PIN_1) - baseline_1;
  previous_flex_2 = analogRead(FLEX_PIN_2) - baseline_2;
  
  //Serial.println("Flex Sensor 1 (bits), Flex Sensor 2 (bits)");

}

void loop() {
  // read the flex sensors
  flex_reading_1 = analogRead(FLEX_PIN_1) - baseline_1;
  flex_reading_2 = analogRead(FLEX_PIN_2) - baseline_2;

  // filter the data 
  // We are applying a filter so we aren't picking up on vibrations 
  filtered_flex_1 = (0.90 * previous_flex_1) + (0.10 * flex_reading_1);
  filtered_flex_2 = (0.90 * previous_flex_2) + (0.10 * flex_reading_2);
  
  // print the data
  Serial.print(flex_reading_1); Serial.print(", ");
  Serial.print(flex_reading_2); Serial.print(", ");
  Serial.print(filtered_flex_1); Serial.print(", ");
  Serial.println(filtered_flex_2);

  // update values
  previous_flex_1 = filtered_flex_1; 
  previous_flex_2 = filtered_flex_2;
  
}
