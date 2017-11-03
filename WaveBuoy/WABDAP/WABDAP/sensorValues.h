#ifndef sensorValues_h__
#define sensorValues_h__

#include <QQuaternion>

class calibrationValues
{
public:
	calibrationValues(void)
	{
		A0 = 1.; A1 = 0.; A2 = 0.;
		A3 = 0.; A4 = 1.; A5 = 0.;
		A6 = 0.; A7 = 0.; A8 = 1.;
		BX = 0; BY = 0; BZ = 0;
		GX = 0; GY = 0; GZ = 0;
		ref = QQuaternion(1,0,0,0);
	}
	~calibrationValues(void)
	{

	}
	double A0, A1, A2, A3, A4, A5, A6, A7, A8;
	double BX, BY, BZ;
	double GX, GY, GZ;
	QQuaternion ref;
};


class sensorValues
{
public:
	sensorValues(void)
	{
		time = GX = GY = GZ = MX = MY = MZ = AX = AY = AZ = 0;
	}
	sensorValues::~sensorValues(void){}

	void operator+=(sensorValues add)
	{
		GX += add.GX;
		GY += add.GY;
		GZ += add.GZ;
		AX += add.AX;
		AY += add.AY;
		AZ += add.AZ;
		MX += add.MX;
		MY += add.MY;
		MZ += add.MZ;
	}
	void operator/=(int N)
	{
		GX /=N;
		GY /=N;
		GZ /=N;
		AX /=N;
		AY /=N;
		AZ /=N;
		MX /=N;
		MY /=N;
		MZ /=N;
	}
	void calibrate(calibrationValues & cal)
	{
		MX = (MX- cal.BX)*cal.A0 + (MY-cal.BY)*cal.A1 + (MZ-cal.BZ)*cal.A2;
		MY = (MX- cal.BX)*cal.A3 + (MY-cal.BY)*cal.A4 + (MZ-cal.BZ)*cal.A5;
		MZ = (MX- cal.BX)*cal.A6 + (MY-cal.BY)*cal.A7 + (MZ-cal.BZ)*cal.A8;
		GX = GX -cal.GX;
		GY = GY -cal.GY;
		GZ = GZ -cal.GZ;
	}

	double time, GX, GY, GZ, MX, MY, MZ, AX, AY, AZ;
};

#endif // sensorValues_h__