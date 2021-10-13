#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

extern Adafruit_BNO055 bno;

void AHRS_update(double *roll, double *pitch, double *yaw)
{
  	imu::Quaternion quat = bno.getQuat();
  	/* IMU is mounted differently from its default orientation. In order to
  	 * avoid singularities at certain angles, we need to rotate the measured
  	 * quaternion before converting it to euler. We do that by multiplying it
  	 * with the magic quaternion below. */
	imu::Quaternion rotate(0.5, -0.5, 0.5, 0.5);

	// Serial.print('(');
	// Serial.print(quat.x(), 4);
	// Serial.print(',');
	// Serial.print(quat.y(), 4);
	// Serial.print(',');
	// Serial.print(quat.z(), 4);
	// Serial.print(',');
	// Serial.print(quat.w(), 4);
	// Serial.println(")");
	
	quat = quat * rotate;

	// Serial.print('(');
	// Serial.print(quat.x(), 4);
	// Serial.print(',');
	// Serial.print(quat.y(), 4);
	// Serial.print(',');
	// Serial.print(quat.z(), 4);
	// Serial.print(',');
	// Serial.print(quat.w(), 4);
	// Serial.println(")");

	/* Axis on the IMU are wierdly flipped so we also need to flip x and z
	 * axis for rotation. When all values are zero, the robot is in neutral
	 * orientation and pointing towards east. */
	imu::Vector<3> quat_euler = quat.toEuler();
	*roll = quat_euler.z()*RAD_TO_DEG;
	*pitch = quat_euler.y()*RAD_TO_DEG;
	*yaw = quat_euler.x()*RAD_TO_DEG;
}