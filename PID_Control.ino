#define SENSOR_1 4
#define SENSOR_2 14
#define SENSOR_3 33
#define SENSOR_4 34
#define SENSOR_5 35
#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal

unsigned long lastTime = 0;  // Initialize lastTime to 0 for calculating time difference
float lastError = 0;        // Initialize lastError to 0 as the initial error
//float weighted = 0;         // Initialize weighted value for the line position calculation

// Sensor positions in mm relative to the left reference point (top view)
int sensor1pos = 20;
int sensor2pos = 36;
int sensor3pos = 50;
int sensor4pos = 65;
int sensor5pos = 80;

// Base Speed and PID constants
int baseSpeed = 110;
float kp = 6.8;  // Proportional constant
float kd = 0.3;  // Derivative constant
float k = 1.0;   // Adjust scaling factor for motor speed

int servoAngle = 0;  // Servo angle
int leftMotorSpeed = 0;  // Left motor speed
int RightMotorSpeed = 0;  // Right motor speed

void setup() {
  Serial.begin(115200);  // Start serial communication
  Wire.begin();          // Join I2C bus (address optional for the master)
}

void loop() {
  unsigned long currentTime = millis();  // Get current time in milliseconds
  unsigned long timeChange = currentTime - lastTime;  // Calculate time difference
  
  // Read sensor values
  int sensor1 = analogRead(SENSOR_1);
  int sensor2 = analogRead(SENSOR_2);
  int sensor3 = analogRead(SENSOR_3);
  int sensor4 = analogRead(SENSOR_4);
  int sensor5 = analogRead(SENSOR_5);

  // Map the sensor values to your desired range (255 to 0)
  sensor1 = map(sensor1, 700, 3940, 255, 0); //700 = brightest 
  sensor2 = map(sensor2, 1008, 4075, 255, 0);
  sensor3 = map(sensor3, 1650, 4095, 255, 0);
  sensor4 = map(sensor4, 0, 3160, 255, 0);
  sensor5 = map(sensor5, 326, 1843, 255, 0);

  // Print the mapped sensor values for debugging
  Serial.print("Sensor1:"); Serial.print(sensor1);
  Serial.print("\t");
  Serial.print("Sensor2:"); Serial.print(sensor2);
  Serial.print("\t");
  Serial.print("Sensor3:"); Serial.print(sensor3);
  Serial.print("\t");
  Serial.print("Sensor4:"); Serial.print(sensor4);
  Serial.print("\t");
  Serial.print("Sensor5:"); Serial.println(sensor5);

  // Weighted average calculation to find line position
  float weighted = sensor1 * sensor1pos + sensor2 * sensor2pos + sensor3 * sensor3pos + sensor4 * sensor4pos + sensor5 * sensor5pos;
  int sum = sensor1 + sensor2 + sensor3 + sensor4 + sensor5;
  int weightedAverage = weighted / sum;  // The position of the line (relative to robot)

  // Calculate error (difference from center of the robot)
  float error = 50 - weightedAverage;  // Assuming 50 is the center position of the sensor array

  // Proportional term
  double PTerm = kp * error; //kp is the propotional constanto

  // Derivative term (calculate rate of change of error)
  double derivative = (error - lastError) / (double)(timeChange * 0.001);  // timeChange in seconds
  double DTerm = (kd * derivative);  // Adjust with Kd

  // Sum output (PID control output)
  double outputU = PTerm + DTerm;

  // Save the current values for the next loop
  lastError = error;
  lastTime = currentTime;

  // Calculate servo and motor speeds
  servoAngle = 90 + outputU;  //  servo angle calculation
  leftMotorSpeed = baseSpeed + (k * outputU);
  RightMotorSpeed = baseSpeed - (k * outputU);

  // Constrain motor speeds to valid PWM values (0-255)
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  RightMotorSpeed = constrain(RightMotorSpeed, -255, 255);

// Send motor speed and servo angle to the slave (send two bytes each)
  Wire.beginTransmission(I2C_SLAVE_ADDR);  // Start communication with the slave
  Wire.write((byte)((leftMotorSpeed >> 8) & 0xFF));  // High byte of left motor speed
  Wire.write((byte)(leftMotorSpeed & 0xFF));  // Low byte of left motor speed
  Wire.write((byte)((RightMotorSpeed >> 8) & 0xFF));  // High byte of right motor speed
  Wire.write((byte)(RightMotorSpeed & 0xFF));  // Low byte of right motor speed
  Wire.write((byte)((servoAngle >> 8) & 0xFF));  // High byte of servo angle
  Wire.write((byte)(servoAngle & 0xFF));  // Low byte of servo angle
  Wire.endTransmission();  // End communication
delay(100);  // Small delay for stability
}





