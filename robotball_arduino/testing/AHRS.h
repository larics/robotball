#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

extern Adafruit_BNO055 bno;
extern const int8_t led_pin;

void AHRS_update(double *roll, double *pitch, double *yaw, double)
{
	/* Get the IMU reading. */
  	imu::Quaternion quat = bno.getQuat();

  	/* IMU is mounted differently from its default orientation. In order to
  	 * avoid singularities at certain angles, we need to rotate the measured
  	 * quaternion before converting it to euler. We do that by multiplying it
  	 * with the magic quaternion below. */
	imu::Quaternion rotate(0.5, -0.5, 0.5, 0.5);

	
	quat = quat * rotate;

	/* Axis on the IMU are wierdly flipped so we also need to flip x and z
	 * axis for rotation. When all values are zero, the robot is in neutral
	 * orientation and pointing towards east. */
	imu::Vector<3> quat_euler = quat.toEuler();

	/* Finally, store the roll, pitch, and yaw. */
	*roll = quat_euler.z()*RAD_TO_DEG;
	*pitch = quat_euler.y()*RAD_TO_DEG;
	*yaw = quat_euler.x()*RAD_TO_DEG;
}

void setup_test_BNO055(void)
{	
	/* Initialize BNO055 IMU. */
	if(!bno.begin())
	{
		// There was a problem detecting the BNO055.
		while(1) {
			digitalWrite(led_pin, LOW);
			delay(500);
			digitalWrite(led_pin, HIGH);
			delay(500);
		}
	}
	delay(1000);
	bno.setExtCrystalUse(true);

	/* Check the calibration status of the IMU. */
	uint8_t system, gyro, accel, mag;
	system = gyro = accel = mag = 0;
	// The data should be ignored until the system calibration is > 0.
	int valid_count = 0;
	while (valid_count < 100)
	{
		bno.getCalibration(&system, &gyro, &accel, &mag);
		valid_count = valid_count * (system > 0) + 1;
	}
	digitalWrite(led_pin, HIGH);
}