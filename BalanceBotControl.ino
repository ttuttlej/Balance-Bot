/*
 * Balance Bot State space Contorller with integral Control
 */

//Libraries
#include<Wire.h> //For I2C communication (Accel/Gyro)
#include<math.h> //for atan2 function

/*
 * Model parameters
 */
float u = 0;
const float Lp = .13;
const float mp = .504;
const float Jp = .002785;
const float r = .033;
const float mw = 2*(.0345);
const float Jw = 2*(.0000276);
const float R = .5*(22);
const float GR = 120.;
const float Kt = .00144167;
const float g = 9.81;    

// Variables designed to simplify notation
d1 = Jp + mp*Lp^2 + mp*Lp*r;
d2 = mp*Lp*r;
d3 = mp*g*Lp;
d4 = Jw +(mp + mw)*r^2 + mp*Lp*r;
d5 = Jw + (mp + mw)*r^2;
d6 = (Kt*GR)^2/R;
d7 = Kt*GR / R;

//State equilibrium values
thetaE = 0;
einE = 0;
einMax = 3*3.8; %3.8 is the max voltage per cell


/*
 * Define all global variables
 *  Accelerometer
 *  Motor
 *  Encoder
 */
// Accelerometer Constants
const float pi = 3.14159;
const float dt = 0.003; //(s) Iteration time step
// Filter coefficients
const float aLow = exp(-dt / 0.33); //low-pass filter coefficient
const float aHi = exp(-dt / 0.05); //high-pass filter coefficient
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //Raw accel gyro data
// Variables that need to be global to keep between time steps
float thetaBias = 0.0;  //Bias in pendulum angle measurement
float dThetaBias = 0.0; //Bias in gyro measurement
unsigned long oldTime = 0; //time in microseconds

//Motor controller pins
// Left motor
const int enL = 11;
const int in1 = 10;
const int in2 = 9;
// Right motor
const int in3 = 8;
const int in4 = 7;
const int enR = 6;
// Variables that need to be global to keep between time steps
unsigned long oldTime = 0; //time in microseconds
float percent = 0.0;  //Desired motor voltage in percent of full battery voltage
int sign = 1; //Direction of desired motor rotation.

// Encoder Constants
const float r = 0.033 / 0.0254; //(in) Wheel radius
const long pulsePerRev = 3840 / 4;
const int ENCLeft  = 3;
const int ENCLDir  = 5;
const int ENCRight = 2;
const int ENCRDir  = 4;
volatile long LeftCount = 0;
volatile long RightCount = 0;
// Variables that need to be global to keep between time steps
float alphaOld = 0.0;   //(rad) previous wheel angle
unsigned long oldTime = 0; //time in microseconds

/*
 * Set up control matrix
 */
//Using the continuous matrices with Euler's method
// Controller gains:
float Ka [4] = { -26.9911, -2.8189, -0.4831, -0.4214};

//Finish initializing pins
void setup() {  
  Serial.begin(115200);

  //ENCODER PINS****
  pinMode(ENCLeft, INPUT_PULLUP);
  pinMode(ENCLDir, INPUT_PULLUP);
  pinMode(ENCRight, INPUT_PULLUP);
  pinMode(ENCRDir, INPUT_PULLUP);
  //set up the interrupts for the motor encoder
  attachInterrupt(digitalPinToInterrupt(ENCLeft), readLeftEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCRight), readRightEncoder, FALLING);
  
  //MOTOR CONTROLLER PINS ***
  // Left motor
  pinMode(enL, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // Right motor
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enR, OUTPUT);
  
  //ACELEROMETER PINS ***
  //set up the code to read the acceleration and rotation
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //determine the bias values
  Serial.print('\n'); Serial.print("Hold Balance-Bot vertical to calibrate sensors.");
  for (int ii = 0; ii < 7000; ii++) {
    // Get the measured accelerometer and gyrometer values
    getMeasurements();
    // Calculate the angle and angular rate bias values using a low-pass filter
    dThetaBias = aLow * dThetaBias + (1.0 - aLow) * ((float)GyY); //(rad/s) angular velocity bias
    thetaBias = aLow * thetaBias + (1.0 - aLow) * (-atan2(((float)AcX) , ((float)AcZ))); //(rad) bias
    if (ii % 1000 == 0) {
      Serial.print("...");
      Serial.print(ii / 1000);
    }
  }
  Serial.print('\n');
}


void loop() { // Measure the angle
  
  
  /*
   * Encoder loop function
   */ 
  //variables that can be deleted each time step
  float alpha;
  float Vel;
  float dist;

  //Get the wheel angle using only the left encoder count
  alpha = LeftCount * 2 * pi / pulsePerRev; //(rad)

  //Get the wheel angular velocity from the change in position
  Vel = (alpha - alphaOld) / dt; //(rad/s) Wheel angular velocity
  alphaOld = alpha;

  //Get the distance traveled for validation
  dist = r * alpha;

  Serial.print(alpha * 180. / pi, 2);
  Serial.print('\t');
  Serial.print(Vel * 180. / pi, 2);
  Serial.print('\t');
  Serial.print(dist, 2);
  Serial.print('\t');
  Serial.print(LeftCount);
  Serial.print('\t');
  Serial.print(RightCount);
  Serial.print('\n');
  while (micros() - oldTime < (long)(dt * 1000000));
  oldTime = micros();
  
  /*
   * Motor loop fcn
   */
  //reverse directions if percent is greater than 100% or less than -100%
  if (abs(percent) >= 1) {
    sign = -sign;
  }
  //change the percent
  percent = percent + 0.001 * sign;
  //call the motor function
  go(percent);

  Serial.println((int)(100 * percent));
  while (micros() - oldTime < (long)(dt * 1000000));
  oldTime = micros();


  /*
   * Acelerometer Loop fcn
   */
  //variables that can be deleted each time step
  float theta;
  float dTheta;

  // Get the measured accelerometer and gyrometer values
  getMeasurements();

  // Calculate theta and dTheta from the accelerometer measurements
  theta = -atan2(((float)AcX) , ((float)AcZ)) - thetaBias; //(rad) pendulum angle
  dTheta = ((float)GyY - dThetaBias) / 131.0 * pi / 180.0; //(rad/s) pendulum anglular rate

  Serial.print(theta * 180. / pi);
  Serial.print('\t');
  Serial.print(dTheta * 180. / pi);
  Serial.print('\n');
  while (micros() - oldTime < (long)(dt * 1000000));
  oldTime = micros();


}



/*
 * Functions for operation
 */

//Function to update the encoder count when the interrupt is triggered,
void readLeftEncoder() {
  LeftCount -= 1 - 2 * digitalRead(ENCLDir);  // -1 if ENCLDir = 0, or +1 if ENCLDir = 1
}
void readRightEncoder() {
  RightCount += 1 - 2 * digitalRead(ENCRDir);  // +1 if ENCRDir = 0, or -1 if ENCRDir = 1
}

//Function to move the motors
void go(float SpeedPercent) {
  float Mag = max(min(abs(SpeedPercent), 0.95), 0.0);
  bool Dir = (SpeedPercent >= 0.);
  // Set the direction according to sign of the speed command
  digitalWrite(in1, Dir);
  digitalWrite(in2, !Dir);
  digitalWrite(in3, Dir);
  digitalWrite(in4, !Dir);
  // Convert to PWM
  int pwm = max(min((int)(26.*exp(2.7175 * abs(Mag))), 220), 0);
  // Write to the motor controller
  analogWrite(enL, pwm);
  analogWrite(enR, pwm);
}

//Function to get accelerometer and gyre data
void getMeasurements() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}


