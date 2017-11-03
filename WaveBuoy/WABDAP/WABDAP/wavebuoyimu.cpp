#include "wavebuoyimu.h"

WaveBuoyIMU::WaveBuoyIMU(QObject *parent)
	: QObject(parent)
{

}

WaveBuoyIMU::~WaveBuoyIMU()
{

}

void WaveBuoyIMU::run()
{
	// Initial conditions:
	// after a few iterations, the gradient descent filter will over-ride the initial conditions
	// so we don't need to know where the sensor starts
	q0 = 1; q1 = 0; q2 = 0; q3 = 0;
	firstIteration = true;
	beta = 0.3;
}

void WaveBuoyIMU::receiveSensorUpdate( sensorValues updateValues )
{

	sensorValues raw = updateValues;
	// Madgwick MARG sensor gradient descent filter
		double recipNorm;
		double s0, s1, s2, s3;
		double qDot1, qDot2, qDot3, qDot4;
		double hx, hy;
		double _2q0MX, _2q0MY, _2q0MZ, _2q1MX, _2bx, _2bz, _4bx, _4bz;
		double _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

		// Rate of change of quaternion from Gyroscope
		qDot1 = 0.5 * (-q1 * updateValues.GX - q2 * updateValues.GY - q3 * updateValues.GZ);
		qDot2 = 0.5 * ( q0 * updateValues.GX + q2 * updateValues.GZ - q3 * updateValues.GY);
		qDot3 = 0.5 * ( q0 * updateValues.GY - q1 * updateValues.GZ + q3 * updateValues.GX);
		qDot4 = 0.5 * ( q0 * updateValues.GZ + q1 * updateValues.GY - q2 * updateValues.GX);

		// Normalize accelerometer
		recipNorm = invSqrt(updateValues.AX * updateValues.AX + updateValues.AY * updateValues.AY + updateValues.AZ * updateValues.AZ);
		updateValues.AX *= recipNorm;
		updateValues.AY *= recipNorm;
		updateValues.AZ *= recipNorm;   

		// Normalize magnetometer
		recipNorm = invSqrt(updateValues.MX * updateValues.MX + updateValues.MY * updateValues.MY + updateValues.MZ * updateValues.MZ);
		updateValues.MX *= recipNorm;
		updateValues.MY *= recipNorm;
		updateValues.MZ *= recipNorm;

		// Pre-define variables for ease of reading (!) - compiler will optimize-out additional memory-space.
		_2q0MX = 2.0 * q0 * updateValues.MX;
		_2q0MY = 2.0 * q0 * updateValues.MY;
		_2q0MZ = 2.0 * q0 * updateValues.MZ;
		_2q1MX = 2.0 * q1 * updateValues.MX;
		_2q0 = 2.0 * q0;
		_2q1 = 2.0 * q1;
		_2q2 = 2.0 * q2;
		_2q3 = 2.0 * q3;
		_2q0q2 = 2.0 * q0 * q2;
		_2q2q3 = 2.0 * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = updateValues.MX * q0q0 - _2q0MY * q3 + _2q0MZ * q2 + updateValues.MX * q1q1 + _2q1 * updateValues.MY * q2 + _2q1 * updateValues.MZ * q3 - updateValues.MX * q2q2 - updateValues.MX * q3q3;
		hy = _2q0MX * q3 + updateValues.MY * q0q0 - _2q0MZ * q1 + _2q1MX * q2 - updateValues.MY * q1q1 + updateValues.MY * q2q2 + _2q2 * updateValues.MZ * q3 - updateValues.MY * q3q3;

		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0MX * q2 + _2q0MY * q1 + updateValues.MZ * q0q0 + _2q1MX * q3 - updateValues.MZ * q1q1 + _2q2 * updateValues.MY * q3 - updateValues.MZ * q2q2 + updateValues.MZ * q3q3;
		_4bx = 2.0 * _2bx;
		_4bz = 2.0 * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 *(2.0 * q1q3 - _2q0q2 - updateValues.AX) + _2q1 * (2.0 * q0q1 + _2q2q3 - updateValues.AY) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - updateValues.MX) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - updateValues.MY) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - updateValues.MZ);
		s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - updateValues.AX) + _2q0 * (2.0 * q0q1 + _2q2q3 - updateValues.AY) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - updateValues.AZ) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - updateValues.MX) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - updateValues.MY) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - updateValues.MZ);
		s2 = -_2q0 *(2.0 * q1q3 - _2q0q2 - updateValues.AX) + _2q3 * (2.0 * q0q1 + _2q2q3 - updateValues.AY) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - updateValues.AZ) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - updateValues.MX) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - updateValues.MY) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - updateValues.MZ);
		s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - updateValues.AX) + _2q2 * (2.0 * q0q1 + _2q2q3 - updateValues.AY) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - updateValues.MX) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - updateValues.MY) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - updateValues.MZ);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalize step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;

		if (firstIteration)
		{
			currentValues = updateValues;
			firstIteration = false;
		}

		// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * (1.0 * (updateValues.time - currentValues.time));
		q1 += qDot2 * (1.0 * (updateValues.time - currentValues.time));
		q2 += qDot3 * (1.0 * (updateValues.time - currentValues.time));
		q3 += qDot4 * (1.0 * (updateValues.time - currentValues.time));

		currentValues = updateValues;

		// Normalize quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;

		currentOrientation = QQuaternion(q0, q1, q2, q3);

		emit(sendQuaternion(currentOrientation));
		emit(sendLogger(raw, currentOrientation));
}

double WaveBuoyIMU::invSqrt( double x )
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return (double)y;
}
