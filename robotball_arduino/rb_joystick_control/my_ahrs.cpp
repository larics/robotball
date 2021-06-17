// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface


/* Sensor libraries and setup */
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM9DS.h"


/* Pick your filter! slower == better quality output */
//Adafruit_NXPSensorFusion filter;  // slowest
//Adafruit_Madgwick filter;         // faster than NXP
Adafruit_Mahony filter;             // fastest/smalleset
#define FILTER_UPDATE_RATE_HZ 30
#define FILTER_USE_RADIANS


/* Sensor calibration data */
#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif


/* Uncomment to enable printing to Serial. */
//#define AHRS_DEBUG_OUTPUT
//#define AHRS_INFO_OUTPUT


/* Global variables */
uint32_t timestamp;
const float deg2rad = 0.0174533;


/* Functions */

void AHRS_setup() 
{
  bool using_serial = false;
#if defined(AHRS_DEBUG_OUTPUT) || defined(AHRS_INFO_OUTPUT)
  Serial.begin(115200);
  while (!Serial) yield();
  using_serial = true;
#endif

  short e = 0;

  if (!cal.begin())
    e = 1;
  else if (! cal.loadCalibration())
    e = 2;

  if (!init_sensors())
    e = 3;
  
#if defined(AHRS_DEBUG_OUTPUT)
  switch (e)
  {
  case 1:
    Serial.println("Failed to initialize calibration helper");
    break;
  case 2:
    Serial.println("No calibration loaded/found");
    break;
  case 3:
    Serial.println("Failed to find sensors");
    break;
  }
#endif

  if (e == 3)
  {
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz

  return using_serial;
}

void AHRS_update(double* roll, double* pitch, double* heading)
{
  float gx, gy, gz;

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gz, -gy, -gx, 
                accel.acceleration.z, -accel.acceleration.y, -accel.acceleration.x, 
                mag.magnetic.z, -mag.magnetic.y, -mag.magnetic.x);
                
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  Serial.print(mag.magnetic.z, 4); Serial.println("");
#endif

  // print the heading, pitch and roll
#if defined(FILTER_USE_RADIANS)
  *roll = filter.getRoll() * deg2rad;
  *pitch = filter.getPitch() * deg2rad;
  *heading = filter.getYaw() * deg2rad;
#else
  *roll = filter.getRoll();
  *pitch = filter.getPitch();
  *heading = filter.getYaw();
#endif
  
#if defined(AHRS_INFO_OUTPUT)
  Serial.print("Orientation: ");
  Serial.print(*heading);
  Serial.print(" ");
  Serial.print(*pitch);
  Serial.print(" ");
  Serial.println(*roll);
#endif
  
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif
}
