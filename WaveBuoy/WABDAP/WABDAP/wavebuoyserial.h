#ifndef WAVEBUOYSERIAL_H
#define WAVEBUOYSERIAL_H

#include <QObject>
#include <QStringList>
#include <QThread>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QVector>
#include "sensorValues.h"


class WaveBuoySerial : public QObject
{
	Q_OBJECT

public:
	WaveBuoySerial(QObject *parent = 0);
	~WaveBuoySerial();

public slots:
	void run();	// this becomes the constructor
	void connectCOM(QString portName);
	void readData();
	void setMovingAverage(int nMovingAverage);
	void disconnectCOM();
	void receiveCalibration(calibrationValues updateCalib)
	{
		calib = updateCalib;
	}

signals:
	void finished();	// emit this when all processing is complete, thread will delete
	void sendPortInfo(QStringList portNames);
	void sendSensorUpdate(sensorValues latestValues);
private slots:

private:
	int nMovingAverage;
	QSerialPort * serial;
	QVector<sensorValues> movingAverageBuffer;
	sensorValues movingAverage;
	calibrationValues calib;
};

#endif // WAVEBUOYSERIAL_H
