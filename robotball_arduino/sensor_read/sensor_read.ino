/* Sensor libraries and setup */
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_LSM9DS1.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
Adafruit_LSM9DS1 lsm9ds1 = Adafruit_LSM9DS1();

bool init_sensors(void) {
  if (!lsm9ds1.begin()) {
    return false;
  }
  accelerometer = &lsm9ds1.getAccel();
  gyroscope = &lsm9ds1.getGyro();
  magnetometer = &lsm9ds1.getMag();

  return true;
}

void setup_sensors(void) {
  // set lowest range
  lsm9ds1.setupAccel(lsm9ds1.LSM9DS1_ACCELRANGE_2G);
  lsm9ds1.setupMag(lsm9ds1.LSM9DS1_MAGGAIN_4GAUSS);
  lsm9ds1.setupGyro(lsm9ds1.LSM9DS1_GYROSCALE_245DPS);
}


/* Sensor calibration data */
#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif



void setup()
{
	Serial.begin(115200);

  if (!cal.begin())
   Serial.println("Failed to initialize calibration helper");
  else if (! cal.loadCalibration())
    Serial.println("No calibration loaded/found");

  if (!init_sensors())
  {
  	Serial.println("Failed to find sensors");
  	digitalWrite(LED_BUILTIN, HIGH);
    while (1) delay(10);
  }

  setup_sensors();

  Wire.setClock(400000); // 400KHz
}

void loop()
{
	sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(",");
  Serial.print(accel.acceleration.y, 4); Serial.print(",");
  Serial.print(accel.acceleration.z, 4); Serial.print(",");
  Serial.print(gyro.gyro.x, 4); Serial.print(",");
  Serial.print(gyro.gyro.y, 4); Serial.print(",");
  Serial.print(gyro.gyro.z, 4); Serial.print(",");
  Serial.print(mag.magnetic.x, 4); Serial.print(",");
  Serial.print(mag.magnetic.y, 4); Serial.print(",");
  Serial.print(mag.magnetic.z, 4); Serial.println("");

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  Serial.print("Calibrated: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(",");
  Serial.print(accel.acceleration.y, 4); Serial.print(",");
  Serial.print(accel.acceleration.z, 4); Serial.print(",");
  Serial.print(gyro.gyro.x, 4); Serial.print(",");
  Serial.print(gyro.gyro.y, 4); Serial.print(",");
  Serial.print(gyro.gyro.z, 4); Serial.print(",");
  Serial.print(mag.magnetic.x, 4); Serial.print(",");
  Serial.print(mag.magnetic.y, 4); Serial.print(",");
  Serial.print(mag.magnetic.z, 4); Serial.println("");

}