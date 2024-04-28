/* This code is for an Drivable Shopping Basket project , It can be controlled
using a bluetooth input of a joystick,string or integer.Wire.h and SoftwareSerial libraries 
come default on arduio and the Adafruit_Sesnor and Adafruit HMC5883 Can be found within the zip*/


//includes required libraries 
#include <Wire.h> //library for I2C communciation
#include <Adafruit_Sensor.h> //Adafruit unified sensor interface library
#include <Adafruit_HMC5883_U.h> //library for HMC5883L compass
#include <SoftwareSerial.h> //library for serial communication on other pins

//compass setup using unique ID
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);


SoftwareSerial gpsSerial(9, 8); //set RX, TX pins for GPS module
String GPSdata; //stores incoming data from GPS
float latitude = 0.0, longitude = 0.0; //variables for latitude and longitude

//bluetooth setup using software serial
const int bluetoothTxPin = 2;
const int bluetoothRxPin = 3;
SoftwareSerial bluetooth(bluetoothRxPin, bluetoothTxPin);

//L298N motor driver pins setup
const int ENA = 6; //pin that controls Speed of left motor
const int IN1 = 10; //pin for the forward direction of left motor
const int IN2 = 11; //pin for the backward direction of left motor
const int ENB = 5; //pin that controls speed of right motor
const int IN3 = 12; //pin for the forward direction of right motor
const int IN4 = 7; //pin for the backward direction of the 

//magnetic delination angle
const float declinationAngle = 0.00553;
//calibration factor for the right motor
const float rightMotorCalibration = 0.78; 

void setup() {
  Serial.begin(9600); //start serial communication with 9600 baurd rate
  mag.begin(); //initialise compass module
  gpsSerial.begin(9600); //start serial communication with GPS
  bluetooth.begin(9600); //start serial communication with Bluetooth

  //set motor driver pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //led pin on 13, set it to high initially to show the system is on
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  //loop every 20ms reading all the component functions
  readCompass();
  readGPS();
  handleBluetooth();
  delay(20);
}

//funtion to read the compass heding data
void readCompass() {
  sensors_event_t event;
  mag.getEvent(&event);
  //uses atan2 function to calculate the heading angle
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  heading += declinationAngle; //uses declination angle to adjust
  if (heading < 0) heading += 2 * PI; //keeps heading between 0 and 360
  if (heading > 2 * PI) heading -= 2 * PI;
  float headingDegrees = heading * 180 / M_PI;//converts heading to degrees
  headingDegrees = headingDegrees - 32; //For calibration after the tests that were done
  Serial.print("Heading (degrees): "); //prints heading to serial monitor for debugging purposes
  Serial.println(headingDegrees);
  //100ms delay
  delay(100);
}

//function to read the GPS data
void readGPS() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read(); //reads the GPS data byte by bye
    if (c == '\n') { //check for neline
      if (GPSdata.startsWith("$GPRMC")) { //check for $GPRMC line
        //calls parse function to extract just the long and lat
        parseGPRMC(GPSdata);
      }
      GPSdata = ""; //clears the string for next line
    } else {
      GPSdata += c; //adds each character to the string
    }
  }
}

void parseGPRMC(String sentence) {
  int firstComma = sentence.indexOf(','); //find the first comma
  int secondComma = sentence.indexOf(',', firstComma + 1); //find the second comma
  int thirdComma = sentence.indexOf(',', secondComma + 1); //find the third comma, which precedes the latitude
  int fourthComma = sentence.indexOf(',', thirdComma + 1); //find the fourth comma, which follows the latitude
  int fifthComma = sentence.indexOf(',', fourthComma + 1); //find the fifth comma, which precedes the longitude
  int sixthComma = sentence.indexOf(',', fifthComma + 1); //find the sixth comma, which follows the longitude

  //extract latitude and longitude from the sentence
  String lat = sentence.substring(thirdComma + 1, fourthComma);
  String lon = sentence.substring(fifthComma + 1, sixthComma);

  //convert raw NMEA latitude and longitude (DDMM.MMMM) to decimal degrees (DD.DDDD)
  latitude = convertToDecimalDegrees(lat);
  longitude = convertToDecimalDegrees(lon);

  //print converted latitude and longitude to Serial Monitor
  Serial.print("Latitude: ");
  Serial.print(latitude, 6); //print with 6 decimal places for debugging
  Serial.print(", Longitude: ");
  Serial.println(longitude, 6); //print with 6 decimal places for debugging
}

//function to convert to decimal degrees from the raw data
float convertToDecimalDegrees(String rawDegrees) {
  float raw = rawDegrees.toFloat(); //convert the rawdegree to float
  int degrees = int(raw / 100); //shift the decimal 2 placed
  float minutes = raw - (degrees * 100); //calculate the minutes 
  return degrees + (minutes / 60); //convert the minutes to decimal and add back the degrees
}

//function to handle bluetooth data
void handleBluetooth() {
  if (bluetooth.available()) { //check for incoming bluetooth data 
    String message = bluetooth.readStringUntil('\n'); //read incoming string until a newline
    if (message.startsWith("@")){ //removes the @ at the start of console comands
      message.remove(0, 1); 
    }
    message.trim(); //remove whitespace
    Serial.println(message); //print to serial monitor for debugging

    //if statements to check what function needs to be called
    if (message.startsWith("J0:")) {//joystick
      handleJoystick(message);
    } else if (message == "B1") {//button for LED
      digitalWrite(13, HIGH);
    } else if (message == "B0") {//button to turn off LED and stop motors
      digitalWrite(13, LOW);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    //if for directions ignoring cases in string
    } else if (message.equalsIgnoreCase("north") || message.equalsIgnoreCase("south") ||
               message.equalsIgnoreCase("east") || message.equalsIgnoreCase("west")) {
      navigateToHeading(message);
    } else if (message.toInt() > 0) { //checks if the message is a positive integer
      float distance = message.toFloat(); //convert string to float
      moveForward(distance); //function to move forward the specified distance in meters
    }
  }
}

void moveForward(float meters) {
  float wheelCircumference = 0.314; //Circumference in meters
  float rotationsNeeded = meters / wheelCircumference;
  int gearRatio = 30; //Gearbox ratio
  float motorSpeedRPM = 140; //RPM under load
  float effectiveRPM = motorSpeedRPM / gearRatio; //effective RPM
  int motorRunTime = (rotationsNeeded / effectiveRPM) * 3600; // Convert RPM to millisecond run time

  //serial monitor printing for debugging
  Serial.print("Rotations Needed: ");
  Serial.println(rotationsNeeded);
  Serial.print("Effective RPM: ");
  Serial.println(effectiveRPM);
  Serial.print("Motor Run Time (ms): ");
  Serial.println(motorRunTime);

  //both motors will spin forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255); //full speed
  analogWrite(ENB, 255); //full speed

  delay(motorRunTime); //run motors for the calculated time

  //stop motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

//function to navigate to a specific compass heading
void navigateToHeading(String direction) {
  direction.toLowerCase(); //convert string to lower case for consistent handling
  //print to serial monitor for debugging
  Serial.print("Navigating to: "); Serial.println(direction);

  float targetHeading = 0; //default to north
  if (direction == "east") {
    targetHeading = 90;
  } else if (direction == "south") {
    targetHeading = 180;
  } else if (direction == "west") {
    targetHeading = 270;
  }

  rotateToHeading(targetHeading);//calls rotate function with desired heading angle
}

//funtion to rotate to the specified heading degress
void rotateToHeading(float targetHeading) {
  sensors_event_t event;
  mag.getEvent(&event); //get lastet event from compass

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float currentHeading = atan2(event.magnetic.y, event.magnetic.x) * 180 / M_PI + declinationAngle;
  if (currentHeading < 0) currentHeading += 360;
  if (currentHeading > 360) currentHeading -= 360;//normalize to 0-360 if greater than 360

  float lowerBound = targetHeading - 20; //Set lower boundary with 20 degree tolerance
  float upperBound = targetHeading + 20; //Set upper boundary with 20 degree tolerance

  //Normalize boundaries stay within 0-360 degrees
  if (lowerBound < 0) lowerBound += 360;
  if (upperBound > 360) upperBound -= 360;

  //difference calculation between the target and heading direction
  float diff = targetHeading - currentHeading;
  if (diff < -180) {
    diff += 360; //helps find the shortest path
  } else if (diff > 180) {
    diff -= 360;
  }

  int speed = 200; //sets  speed
  bool isClockwise = diff > 0; //deterime true or false on the rotation direction

  //rotate in the appropriate direction
  if (isClockwise) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  //applys speed to motors
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);

  //Continue rotating until heading is aligned within the range
  do {
    mag.getEvent(&event); //update heading event from compass
    currentHeading = atan2(event.magnetic.y, event.magnetic.x) * 180 / M_PI + declinationAngle;
    if (currentHeading < 0) currentHeading += 360; //normalise heading
    if (currentHeading > 360) currentHeading -= 360;

    //check heading to see if its in range
    if (lowerBound < upperBound) {
      if (currentHeading >= lowerBound && currentHeading <= upperBound) break;
    } else { //case to handle wrapping around zero degrees
      if (currentHeading >= lowerBound || currentHeading <= upperBound) break;
    }

    delay(100);
  } while (true); //keep the roation until range is reached (may take multiple iterations)

  //Stop motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

//function to handle joystick input
void handleJoystick(String data) {
  //find the pos of comma and colon to parse the wanted data
  int separatorIndex = data.indexOf(',');
  int colonIndex = data.indexOf(':');
  
  //check if both datasets were found
  if (separatorIndex > -1 && colonIndex > -1) {
    //extract angle and convert it to an integer
    int angle = data.substring(colonIndex + 1, separatorIndex).toInt();
    //extract speed factor converting it to a float for PWM
    float speedFactor = data.substring(separatorIndex + 1).toFloat();
    
    //calc motor speeds by scaling the speed factor for PWM range
    int leftMotorSpeed = int(speedFactor * 255);  //Scale speed factor to PWM duty cycle
    int rightMotorSpeed = int(speedFactor * 255 * rightMotorCalibration);  //Apply calibration and scale speed

    //write the calculated speeds to the motor driver pins
    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);

    //45-135 forward function
    if (angle > 45 && angle < 135) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if (angle > 225 && angle < 315) { //225-315 for reverse function
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else if ((angle > 135 && angle < 225) || angle > 315 || angle < 45) {
      if (angle >= 180) { //Turn left function
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      } else { //turn right function
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
      }
    }
  }
}
