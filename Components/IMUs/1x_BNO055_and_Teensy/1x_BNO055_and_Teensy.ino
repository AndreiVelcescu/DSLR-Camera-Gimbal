// Copyright (c) Adafruit Industries.
// Based on the Adafruit's BNO055 example.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (0)

// Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BNO055 bnoA = Adafruit_BNO055(-1, BNO055_ADDRESS_A, &Wire);
// Adafruit_BNO055 bnoB = Adafruit_BNO055(-1, BNO055_ADDRESS_B);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(Adafruit_BNO055 *device)
{
  sensor_t sensor;
  device->getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(Adafruit_BNO055 *device)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  device->getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(5);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(Adafruit_BNO055 *device)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  device->getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup()
{ // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(230400);
  Serial.println(BNO055_ADDRESS_A);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bnoA.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, BNO055(A) not detected");
    while(1);
  }
//  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails(&bnoA);
  /* Optional: Display current status */
  displaySensorStatus(&bnoA);
  bnoA.setExtCrystalUse(true);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop()
{ digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  int t=micros();
  
  /* Get a new sensor event */
  sensors_event_t eventA;
  bnoA.getEvent(&eventA);
  /* Display the floating point data */
  // Serial.print("Za");
  // Serial.print(eventA.orientation.x, 4);
  // Serial.print("\t");
  // Serial.print(eventA.orientation.y, 4);
  // Serial.print("\t");
  delayMicroseconds(4985-(micros()-t));
  Serial.print(micros()-t);
  Serial.print("\t");
  Serial.println(eventA.orientation.z, 2);
  

  
  // delayMicroseconds((micros()-t));
  // digitalWrite(LED_BUILTIN, LOW);  // turn the LED off (LOW is the voltage level)
  
}

