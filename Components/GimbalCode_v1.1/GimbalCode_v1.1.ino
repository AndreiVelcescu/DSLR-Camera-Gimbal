#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>
#include <AS5048A.h>
#include <math.h>
#include "variables.h"

// Create an IntervalTimer objects
IntervalTimer myTimer;
IntervalTimer myTimer1;
IntervalTimer myTimer2;
IntervalTimer myTimer3;

// Initialise Encoders
AS5048A angleSensor1(9, false);
AS5048A angleSensor2(10, false);
AS5048A angleSensor3(15, false);

// Set up BNO055 Gyroscopes
Adafruit_BNO055 bno = Adafruit_BNO055(WIRE_BUS, -1, BNO055_ADDRESS_A, I2C_MASTER, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400, I2C_OP_MODE_ISR);
Adafruit_BNO055 bno2 = Adafruit_BNO055(WIRE_BUS, -1, BNO055_ADDRESS_B, I2C_MASTER, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400, I2C_OP_MODE_ISR);

////////////////////////////////////////////////////////////////////////////////////////////////
// Declare Variables ///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

volatile float encAngle1 = 0, encAngle2 = 0, encAngle3 = 0, t0=0,  dA=0;
volatile int u_array1[]= {1,1}, u_array2[]= {1,1};

///////////////////////////////////////////////////////////////////////////////////////////////
/////////////// SetUP /////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  
  // Initialise Encoders 
  angleSensor1.begin();
  angleSensor2.begin();
  angleSensor3.begin();

  // Set Timer Interupt Periods
  myTimer.begin(ReadEncInt, 150);       // run every 100 microseconds
  myTimer1.begin(DriveInt, 150);        // run every 250 microseconds
  myTimer2.begin(ControlInt, 300);      // run every 500 microseconds
  myTimer3.begin(derivativeInt, 1000);  // run every 1000 microseconds

  //////////////PWM SetUp ////////////////////////////
  analogWriteResolution(12);
  pinMode(Pin_A1, OUTPUT);
  analogWriteFrequency(Pin_A1, 50000);
  pinMode(Pin_B1, OUTPUT);
  analogWriteFrequency(Pin_B1, 50000);
  pinMode(Pin_C1, OUTPUT);
  analogWriteFrequency(Pin_C1, 50000);

  pinMode(Pin_A2, OUTPUT);
  analogWriteFrequency(Pin_A2, 50000);
  pinMode(Pin_B2, OUTPUT);
  analogWriteFrequency(Pin_B2, 50000);
  pinMode(Pin_C2, OUTPUT);
  analogWriteFrequency(Pin_C2, 50000);

  // Enable MOS - Motor X
  pinMode(LowMos, OUTPUT);
  digitalWrite(LowMos, HIGH);

  // Osciloscope output - test pins
  pinMode(pin0, OUTPUT);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
}
///////////////////////////////////////////////////////////////////////////////////////////////


/////////////// Motor Drive //////////////////////////////////////////////////////////////////
void driveMotor(float enc, int Pin_A, int Pin_B, int Pin_C, volatile int *u_array){
  static volatile float phase_A = 0, phase_B = 0, phase_C = 0;
  static volatile int AngleA = 0, AngleB = 0, AngleC = 0;

  AngleA = enc * 10;

  AngleB = AngleA + 1200;
  if (AngleB > 3600) {
    AngleB = AngleB - 3600;
  }
  if (AngleB < 0) {
    AngleB = AngleB + 3600;
  }

  AngleC = AngleA + 2400;
  if (AngleC > 3600) {
    AngleC = AngleC - 3600;
  }
  if (AngleC < 0) {
    AngleC = AngleC + 3600;
  }

  if (u_array[0] < 0) {
    phase_A = Phase_ABC[AngleA];
    phase_B = Phase_ABC[AngleB];
    phase_C = Phase_ABC[AngleC];
  } else {
    phase_A = 1.0 - Phase_ABC[AngleA];
    phase_B = 1.0 - Phase_ABC[AngleB];
    phase_C = 1.0 - Phase_ABC[AngleC];
  }

  analogWrite(Pin_A, u_array[1] * phase_A);
  analogWrite(Pin_B, u_array[1] * phase_B);
  analogWrite(Pin_C, u_array[1] * phase_C);
}
///////////////////////////////////////////////////////////////////////////////////////////////


/////////////// Motor Control ////////////////////////////////////////////////////////////////
void motorControl(volatile int *u_array, volatile float reference, volatile float angle){
  volatile int u = 1, abs_u = 0;
  volatile float error = 0;

  error = reference - angle; //spin
  if (error > 180) {
    error = 360 - error;
  } else if (error < -180) {
    error = 360 + error;
  }
  
  u = 50 * error;
  abs_u = abs(u);

  if (abs_u > 4095)
    abs_u = 4095;

  u_array[0] = u;
  u_array[1] = abs_u;
}
///////////////////////////////////////////////////////////////////////////////////////////////


/////////////// Interrupt Routines ///////////////////////////////////////////////////////////

void ReadEncInt() {
  digitalWrite(pin0, HIGH);
  encAngle1 = angleSensor1.getRotationInDegrees();
  encAngle2 = angleSensor2.getRotationInDegrees();
  encAngle3 = angleSensor3.getRotationInDegrees();
  digitalWrite(pin0, LOW);
}

void DriveInt() {
  digitalWrite(pin1, HIGH);
  driveMotor(encAngle3, Pin_A1, Pin_B1, Pin_C1, u_array1);
  // driveMotor(encAngle1, Pin_A2, Pin_B2, Pin_C2, u_array2);
  digitalWrite(pin1, LOW);
}

void ControlInt() {
  digitalWrite(pin2, HIGH);
  motorControl(u_array1,count,encAngle3);
  // motorControl(u_array2,count,encAngle1);
  digitalWrite(pin2, LOW);
}

volatile float pre_Enc3 = 0, dEnc = 0;
volatile int t_prev =0, dT = 0, an = 0;
void derivativeInt(){
  an = encAngle3*10;
  dEnc = 100000*(an - pre_Enc3)/(micros()-t_prev);
  // dT = micros()-t_prev;
  t_prev = micros();
  pre_Enc3 = an; 
}
///////////////////////////////////////////////////////////////////////////////////////////////

/////////////// Main Loop ////////////////////////////////////////////////////////////////////
int ok = 0;
void loop() {
  // count = count + 1;
  // // count2 = count2 + 1;
  
  // if (count > 360){
  //   count = 0;
  // }

  // use this to test Encoder derivative vs BNO055 derivative
  if (count <= 0){
    ok = 1;
  }
  else if (count >= 90){
    ok = 0;
  }  

  if (ok == 1){
    count = count + 1;
  }
  else{
    count = count - 1;
  }
  //////////////////////////////////////////////////////////

  Serial.print(u_array1[0]);
  Serial.print("\t");
  Serial.print(dEnc);
  Serial.print("\t");
  Serial.print(encAngle1);
  Serial.print("\t");
  Serial.print(encAngle2); 
  Serial.print("\t");  
  Serial.println(encAngle3);
  delay(2);
}
///////////////////////////////////////////////////////////////////////////////////////////////