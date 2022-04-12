#include <Wire.h>

/*
 * PID gain settings / limits
 */
float kp_Roll = 0.7;
float ki_Roll = 0.04;
float kd_Roll = 10;
int pidMax_Roll = 400;

float kp_Pitch = kp_Roll;
float ki_Pitch  = ki_Roll;
float kd_Pitch = kd_Roll;
int pidMax_Pitch = pidMax_Roll;

float kp_Yaw = 3;
float ki_Yaw = 0.01;
float kd_Yaw = 0;
int pidMax_Yaw = 400;

/*
 * Variable declarations
 */
boolean gyroAnglesSet;
byte lastCh_1, lastCh_2, lastCh_3, lastCh_4;
volatile int rcInCh_1, rcInCh_2, rcInCh_3, rcInCh_4;
int rcInput[5];
int cCh_1, cCh_2, cCh_3, cCh_4, cCal, cLoop;  // counters
int motor_1, motor_2, motor_3, motor_4;
int start, throttle;
const int IMU_Add = 0x68;
unsigned long elapsedTime, currentTime, previousTime;
unsigned long timer_1, timer_2, timer_3, timer_4;
unsigned long timerCh_1, timerCh_2, timerCh_3, timerCh_4, loop_timer, esc_loop_timer;
long acc_X, acc_Y, acc_Z, accVector;
double gyro_X, gyro_Y, gyro_Z;
float accAngle_X, accAngle_Y;
float gyroAngle_X, gyroAngle_Y, gyroAngle_Z;
double accOffset_X, accOffset_Y;
double gyroOffset_X, gyroOffset_Y, gyroOffset_Z;
float adjust_Roll, adjust_Pitch;
float angle_Roll, angle_Pitch, angle_Yaw;

float pidIn_Roll, pidIn_Pitch, pidIn_Yaw;
float pidError;
float pidSetpoint_Roll, pidSetpoint_Pitch, pidSetpoint_Yaw;
float pidISum_Roll, pidISum_Pitch, pidISum_Yaw;
float pidDLastError_Roll, pidDLastError_Pitch, pidDLastError_Yaw;
float pidOut_Roll, pidOut_Pitch, pidOut_Yaw;

/*
 * Setup
 */
void setup() {
  Serial.begin(57600);
  Wire.begin();

  TWBR = 12;                                                // Set I2C clock speed to 400kHz

  DDRD |= B11110000;                                        // Set ports 4/5/6/7 as output
  DDRB |= B00110000;                                        // Set ports 12/13 as output
  
  setupIMU();                                               // Set up IMU registers

  // Send pulse to ESC while calibrating
  for (cCal = 0; cCal < 1250 ; cCal ++) {
    PORTD |= B11110000;                                     // Set ports 4/5/6/7 to HIGH
    delay(1);
    PORTD &= B00001111;                                     // Set ports 4/5/6/7 to LOW
    delay(3);
  }
  
  calibrateIMU();                                           // Calibrate IMU offsets

  // Interrupt setup
  PCICR |= (1 << PCIE0);                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                  //Set PCINT0 (digitalIN 8) to trigger interrupt on state change
  PCMSK0 |= (1 << PCINT1);                                  //Set PCINT1 (digitalIN 9) to trigger interrupt on state change
  PCMSK0 |= (1 << PCINT2);                                  //Set PCINT2 (digitalIN 10) to trigger interrupt on state change
  PCMSK0 |= (1 << PCINT3);                                  //Set PCINT3 (digitalIN 11) to trigger interrupt on state change

  
  // Wait for RC signal and low throttle
  while(rcInCh_3 < 990 || rcInCh_3 > 1020 || rcInCh_4 < 1400) {
    rcInCh_3 = convertRC(3);                                // Scale RC throttle signal
    rcInCh_4 = convertRC(4);                                // Scale RC yaw signal

    // Pulse ESC while waiting
    PORTD |= B11110000;                                     // Set ports 4/5/6/7 to HIGH
    delay(1);
    PORTD &= B00001111;                                     // Set ports 4/5/6/7 to LOW
    delay(3);  
  }
  
  start = 0;                                                // Set start to 0

  loop_timer = micros();                                    // Set loop timer
}

/*
 * Main program loop
 */
void loop() {
  // PID inputs in deg/sec
  pidIn_Roll = (pidIn_Roll * 0.7) + ((angle_Roll / 65.5) * 0.3);
  pidIn_Pitch = (pidIn_Pitch * 0.7) + ((angle_Pitch / 65.5) * 0.3);
  pidIn_Yaw = (pidIn_Yaw * 0.7) + ((angle_Yaw / 65.5) * 0.3);

  // Calculate Gryo angles
  gyroAngle_X = gyroAngle_X + (gyro_X * 0.004) / 65.5;
  gyroAngle_Y = gyroAngle_Y + (gyro_Y * 0.004) / 65.5;
  gyroAngle_Z = gyroAngle_Z + (gyro_Z * 0.004) / 65.5;

  //0.01745 = PI / 180deg
  gyroAngle_X += gyroAngle_Y * sin(gyroAngle_Z * 0.01745);      // If IMU has yawed, transfer Y angle to X
  gyroAngle_Y -= gyroAngle_X * sin(gyroAngle_Z * 0.01745);      // If IMU has yawed, transfer X angle to Y

  // Calculate Accel angles
  accVector = sqrt((acc_X * acc_X) + (acc_Y * acc_Y) + (acc_Z * acc_Z));            // Total accel vector

  // 57.296 = 1 / (PI / 180deg)
  if(abs(acc_X) < accVector) {                                  // Prevents NaN
    accAngle_X = asin((float)acc_X/accVector) * -57.296;        // Roll angle calculation
  }

  if(abs(acc_Y) < accVector) {                                  // Prevents NaN
    accAngle_Y = asin((float)acc_Y/accVector) * 57.296;        // Pitch angle calculation
  }

  // Complementary filter - Correct Gyro drift with Accel data
  gyroAngle_X = 0.96 * gyroAngle_X + 0.04 * accAngle_X;
  gyroAngle_Y = 0.96 * gyroAngle_Y + 0.04 * accAngle_Y;

  // RPY Angles
  angle_Roll = gyroAngle_X;
  angle_Pitch = gyroAngle_Y;
  angle_Yaw = gyroAngle_Z;
  
  // Calculate angle corrections
  adjust_Roll = angle_Roll * 15;
  adjust_Pitch = angle_Pitch * 15;


  
  // Arm motor (Throttle low, yaw left)
  if(rcInCh_3 < 1050 && rcInCh_4 < 1050) start = 1;
  
  // Start motor (Throttle low, yaw center)
  if(start == 1 && rcInCh_3 < 1050 && rcInCh_4 > 1450) {
    start = 2;

    angle_Roll = accAngle_X;
    angle_Pitch = accAngle_Y;
    gyroAnglesSet = true;

    // Reset PID controller memory
    pidISum_Roll = 0;
    pidDLastError_Roll = 0;
    pidISum_Pitch = 0;
    pidDLastError_Pitch = 0; 
    pidISum_Yaw = 0;
    pidDLastError_Yaw = 0;
  }
   
  // Stop motor (Throttle low, yaw right)
  if(start == 2 && rcInCh_3 < 1050 && rcInCh_4 > 1950) start = 0;

  // Get PID Roll setpoint from RC input
  pidSetpoint_Roll = 0;

  // Add +/- 8us deadband
  if(rcInCh_1 > 1508) pidSetpoint_Roll = rcInCh_1 - 1508;
  else if(rcInCh_1 < 1492) pidSetpoint_Roll = rcInCh_1 - 1492;

  pidSetpoint_Roll -= adjust_Roll;                // Subtract angle correction from RC Input    
  pidSetpoint_Roll /= 3.0;                        // Divide by 3 for angles in deg (Max roll rate ~164deg/s) 
  
  // Get PID Roll setpoint from RC input
  pidSetpoint_Pitch = 0;

 // Add +/- 8us deadband
  if(rcInCh_2 > 1508) pidSetpoint_Pitch = rcInCh_2 - 1508;
  else if(rcInCh_2 < 1492) pidSetpoint_Pitch = rcInCh_2 - 1492;

  pidSetpoint_Pitch -= adjust_Pitch;               // Subtract angle correction from RC Input                    
  pidSetpoint_Pitch /= 3.0;                        // Divide by 3 for angles in deg (Max pitch rate ~164deg/s)
  
  // Get PID Roll setpoint from RC input
  pidSetpoint_Yaw = 0;

  // Don't yaw when stopping motors
  if(rcInCh_3 > 1050) {
    // Add +/- 8us deadband
    if(rcInCh_4 > 1508) pidSetpoint_Yaw = (rcInCh_4 - 1508) / 3.0;
    else if(rcInCh_4 < 1492) pidSetpoint_Yaw = (rcInCh_4 - 1492) / 3.0;
  }

  // Calculate PID controller adjustments
  calculate_pid();

  // Base input
  throttle = rcInCh_3;

  // Motors started
  if (start == 2) {
    if (throttle > 1800) throttle = 1800;                         // Limit throttle to allow for adjustment
    motor_1 = throttle + pidOut_Roll - pidOut_Pitch - pidOut_Yaw; // Motor 1 (RF - CCW)
    motor_2 = throttle + pidOut_Roll + pidOut_Pitch + pidOut_Yaw; // Motor 2 (RR - CW)
    motor_3 = throttle - pidOut_Roll + pidOut_Pitch - pidOut_Yaw; // Motor 3 (LR - CCW)
    motor_4 = throttle - pidOut_Roll - pidOut_Pitch + pidOut_Yaw; // Motor 4 (LF - CW)

    // Limit min pulse to keep motors running
    if (motor_1 < 1100) motor_1 = 1100;
    if (motor_2 < 1100) motor_2 = 1100;
    if (motor_3 < 1100) motor_3 = 1100;
    if (motor_4 < 1100) motor_4 = 1100;

    // Limit max pulse
    if (motor_1 > 2000) motor_1 = 2000;
    if (motor_2 > 2000) motor_2 = 2000;
    if (motor_3 > 2000) motor_3 = 2000;
    if (motor_4 > 2000) motor_4 = 2000;
  }

  // Send pulse while waiting for start
  else {
    motor_1 = 1000;
    motor_2 = 1000;
    motor_3 = 1000;
    motor_4 = 1000;
  }

  // Make sure loop time = 4000us
  while(micros() - loop_timer < 4000);
  loop_timer = micros();

  // Set ports 4/5/6/7 (motors) to HIGH
  PORTD |= B11110000;
  timerCh_1 = motor_1 + loop_timer;                         // Calculate pulse time
  timerCh_2 = motor_2 + loop_timer;
  timerCh_3 = motor_3 + loop_timer;
  timerCh_4 = motor_4 + loop_timer;

  // Get IMU data / RC signal while waiting
  getIMUData();

  while(PORTD >= 16) {                                      // While ports 4/5/6/7 are HIGH
    esc_loop_timer = micros();
    if(timerCh_1 <= esc_loop_timer) PORTD &= B11101111;     // Set port 4 to LOW after pulse time
    if(timerCh_2 <= esc_loop_timer) PORTD &= B11011111;     // Set port 5 to LOW after pulse time
    if(timerCh_3 <= esc_loop_timer) PORTD &= B10111111;     // Set port 6 to LOW after pulse time
    if(timerCh_4 <= esc_loop_timer) PORTD &= B01111111;     // Set port 7 to LOW after pulse time
  }
}

/*
 * Interrupt set up
 */
ISR(PCINT0_vect) {
  currentTime = micros();
  //Channel 1
  if(PINB & B00000001) {                                    // If input 8 is high
    if(lastCh_1 == 0) {                                     // State change from 0 to 1
      lastCh_1 = 1;                                         // Save current state
      timer_1 = currentTime;                                // Set timer to current time
    }
  }
  else if(lastCh_1 == 1) {                                  // If input 8 is not high and changed from 1 to 0
    lastCh_1 = 0;                                           // Save current state    
    rcInput[1] = currentTime - timer_1;                     // Ch 1 = current time - timer_1
  }
  //Channel 2
  if(PINB & B00000010) {                                    // If input 9 is high
    if(lastCh_2 == 0) {                                     // State change from 0 to 1
      lastCh_2 = 1;                                         // Save current state
      timer_2 = currentTime;                                // Set timer to current time
    }
  }
  else if(lastCh_2 == 1) {                                  // If input 9 is not high and changed from 1 to 0
    lastCh_2 = 0;                                           // Save current state    
    rcInput[2] = currentTime - timer_2;                     // Ch 2 = current time - timer_2
  }
  //Channel 3
  if(PINB & B00000100) {                                    // If input 10 is high
    if(lastCh_3 == 0) {                                     // State change from 0 to 1
      lastCh_3 = 1;                                         // Save current state
      timer_3 = currentTime;                                // Set timer to current time
    }
  }
  else if(lastCh_3 == 1) {                                  // If input 10 is not high and changed from 1 to 0
    lastCh_3 = 0;                                           // Save current state    
    rcInput[3] = currentTime - timer_3;                     // Ch 3 = current time - timer_3
  }
  //Channel 4
  if(PINB & B00001000) {                                    // If input 11 is high
    if(lastCh_4 == 0) {                                     // State change from 0 to 1
      lastCh_4 = 1;                                         // Save current state
      timer_4 = currentTime;                                // Set timer to current time
    }
  }
  else if(lastCh_4 == 1) {                                  // If input 11 is not high and changed from 1 to 0
    lastCh_4 = 0;                                           // Save current state    
    rcInput[4] = currentTime - timer_4;                     // Ch 4 = current time - timer_4
  }
}

/*
 * Set up IMU registers
 */
void setupIMU() {
  Wire.beginTransmission(IMU_Add);                          // Start communication with IMU
  Wire.write(0x6B);                                         // Access 0x6B (PWR_MGMT_1)
  Wire.write(0b00);                                         // Set SLEEP register to 0
  Wire.endTransmission();

  Wire.beginTransmission(IMU_Add);                          // Start communication with IMU
  Wire.write(0x1B);                                         // Access 0x1B (GYRO_CONFIG)
  Wire.write(0x08);                                         // Set Gyro to +/- 500deg/s full scale
  Wire.endTransmission();

  Wire.beginTransmission(IMU_Add);                          // Start communication with IMU
  Wire.write(0x1C);                                         // Access 0x1C (ACCEL_CONFIG)
  Wire.write(0x10);                                         // Set Accel to +/- 8g full scale
  Wire.endTransmission();
}

/*
 * Calibrate IMU offsets
 */
void calibrateIMU() {
  for (cCal = 0; cCal < 2000; cCal++) {                     // Take 2000 samples
    getIMUData();
    accOffset_X += acc_X;
    accOffset_Y += acc_Y;

    gyroOffset_X += gyro_X;
    gyroOffset_Y += gyro_Y;
    gyroOffset_Z += gyro_Z;

    // Pulse ESC to avoid beeping
    PORTD |= B11110000;                                     // Set ports 4/5/6/7 to HIGH
    delay(1);
    PORTD &= B00001111;                                     // Set ports 4/5/6/7 to LOW
    delay(3);  
  }

  accOffset_X /= 2000;                                      // Divide by 2000 to get average
  accOffset_Y /= 2000;

  gyroOffset_X /= 2000;
  gyroOffset_Y /= 2000;
  gyroOffset_Z /= 2000;
}

/*
 * Get IMU Data / RC Input
 */
void getIMUData() {
  // Get Accel data
  Wire.beginTransmission(IMU_Add);                          // Start communication with IMU
  Wire.write(0x3B);                                         // Starting register for Accel reading
  Wire.endTransmission();

  Wire.requestFrom(IMU_Add,6);                              // Request Accel registers (3B - 40)
  while(Wire.available() < 6);
  acc_Y = (Wire.read()<<8|Wire.read());                     // First two bytes -> acc_Y (Along roll axis)
  acc_X = (Wire.read()<<8|Wire.read());                     // Middle two bytes -> acc_X (Along pitch axis)
  acc_Z = (Wire.read()<<8|Wire.read());                     // Last two bytes -> acc_Z

  // Get Gyro data
  Wire.beginTransmission(IMU_Add);                          // Start communication with IMU
  Wire.write(0x43);                                         // Starting register for Gyro reading
  Wire.endTransmission();

  Wire.requestFrom(IMU_Add,6);                              // Request Gyro registers (43 - 48)
  while(Wire.available() < 6);
  gyro_X = (Wire.read()<<8|Wire.read());                    // First two bytes -> gyro_X
  gyro_Y = (Wire.read()<<8|Wire.read());                    // First two bytes -> gyro_Y
  gyro_Z = (Wire.read()<<8|Wire.read());                    // First two bytes -> gyro_Z

  // Scale RC inputs to 1000-2000us
  rcInCh_1 = convertRC(1);
  rcInCh_2 = convertRC(2);
  rcInCh_3 = convertRC(3);
  rcInCh_4 = convertRC(4);
  
  if (cCal == 2000) {                                       // Adjust with offsets
    acc_X -= accOffset_X;
    acc_Y -= accOffset_Y;
  
    gyro_X -= gyroOffset_X;
    gyro_Y -= gyroOffset_Y;
    gyro_Z -= gyroOffset_Z;
  }

  // Invert due to IMU mounting orientation
  acc_Y *= -1;                                              // Invert Y axis
  acc_Z *= -1;                                              // Invert Z axis

  gyro_X *= -1;                                             // Invert X axis
  gyro_Z *= -1;                                             // Invert Z axis
}

/*
 * PID Calculations
 */
void calculate_pid() {
  // Roll
  pidError = pidIn_Roll - pidSetpoint_Roll;
  pidISum_Roll += ki_Roll * pidError;
  if(pidISum_Roll > pidMax_Roll) pidISum_Roll = pidMax_Roll;
  else if(pidISum_Roll < pidMax_Roll * -1) pidISum_Roll = pidMax_Roll * -1;

  pidOut_Roll = kp_Roll * pidError + pidISum_Roll + kd_Roll * (pidError - pidDLastError_Roll);
  if(pidOut_Roll > pidMax_Roll) pidOut_Roll = pidMax_Roll;
  else if(pidOut_Roll < pidMax_Roll * -1) pidOut_Roll = pidMax_Roll * -1;

  pidDLastError_Roll = pidError;

  // Pitch
  pidError = pidIn_Pitch - pidSetpoint_Pitch;
  pidISum_Pitch += ki_Pitch * pidError;
  if(pidISum_Pitch > pidMax_Pitch) pidISum_Pitch = pidMax_Pitch;
  else if(pidISum_Pitch < pidMax_Pitch * -1) pidISum_Pitch = pidMax_Pitch * -1;

  pidOut_Pitch = kp_Pitch * pidError + pidISum_Pitch + kd_Pitch * (pidError - pidDLastError_Pitch);
  if(pidOut_Pitch > pidMax_Pitch) pidOut_Pitch = pidMax_Pitch;
  else if(pidOut_Pitch < pidMax_Pitch * -1) pidOut_Pitch = pidMax_Pitch * -1;

  pidDLastError_Pitch = pidError;

  // Yaw
  pidError = pidIn_Yaw - pidSetpoint_Yaw;
  pidISum_Yaw += ki_Yaw * pidError;
  if(pidISum_Yaw > pidMax_Yaw) pidISum_Yaw = pidMax_Yaw;
  else if(pidISum_Yaw < pidMax_Yaw * -1) pidISum_Yaw = pidMax_Yaw * -1;

  pidOut_Yaw = kp_Yaw * pidError + pidISum_Yaw + kd_Yaw * (pidError - pidDLastError_Yaw);
  if(pidOut_Yaw > pidMax_Yaw) pidOut_Yaw = pidMax_Yaw;
  else if(pidOut_Yaw < pidMax_Yaw * -1) pidOut_Yaw = pidMax_Yaw * -1;

  pidDLastError_Yaw = pidError;
}

/*
 * Scale RC input to standardized 1000-2000us
 */
int convertRC(byte channel){
  int high[5] = {0, 2008, 2008, 2008, 2004};                        // Measured max values from RC
  int low[5] = {0, 1000, 1012, 1000, 1000};                         // Measured min values from RC
  int hi, center, lo, actual;
  int diff;
  
  center = 1500;
 
  actual = rcInput[channel];
  hi = high[channel];
  lo = low[channel];

  if(actual < center) {
    if(actual < lo) actual = lo;                                    // Limit to min value
    diff = ((long)(center - actual) * (long)500) / (center - lo);   // Scale value to 1000-2000us
    if(channel == 2) return 1500 + diff;                            // Invert for Channel 2 (Pitch/Y Axis)
    else return 1500 - diff;
  }

  else if(actual > center) {
    if(actual > hi) actual = hi;                                     // Limit to max value
    diff = ((long)(actual - center) * (long)500) / (hi - center);    // Scale value to 1000-2000us
    if(channel == 2) return 1500 - diff;                             // Invert for Channel 2 (Pitch/Y Axis)
    else return 1500 + diff;
  }

  else return 1500;
}
