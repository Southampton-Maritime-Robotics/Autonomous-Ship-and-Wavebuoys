#ifndef WAVEBUOYIMU_H
#define WAVEBUOYIMU_H

#include <QObject>
#include <QQuaternion>
#include "sensorValues.h"

class WaveBuoyIMU : public QObject
{
	Q_OBJECT

public:
	WaveBuoyIMU(QObject *parent = 0);
	~WaveBuoyIMU();

public slots:
	void run();;	// this becomes the constructor
	void receiveSensorUpdate( sensorValues updateValues );
signals:
	void finished();	// emit this when all processing is complete, thread will delete
	void sendQuaternion(QQuaternion updateQ);
	void sendLogger(sensorValues updateValues, QQuaternion currentOrientation);
private:
	double invSqrt(double x);
	sensorValues currentValues;
	double q0, q1, q2, q3;
	double beta;
	QQuaternion currentOrientation;
	bool firstIteration;
};

#endif // WAVEBUOYIMU_H
