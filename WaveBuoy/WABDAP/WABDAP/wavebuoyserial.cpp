#include "wavebuoyserial.h"

WaveBuoySerial::WaveBuoySerial(QObject *parent)
	: QObject(parent)
{

}

WaveBuoySerial::~WaveBuoySerial()
{
	delete serial;
}

void WaveBuoySerial::run()
{
	// Create the serial port manager and update the GUI
	serial = new QSerialPort();
	QStringList portNames;
	foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
		portNames.append(info.portName());
	emit(sendPortInfo(portNames));

	setMovingAverage(5);
}

void WaveBuoySerial::connectCOM( QString portName )
{
	// Connects the serial port
	if (serial->isOpen())
		serial->close();
	serial->setPortName(portName);
	serial->open(QIODevice::ReadOnly);
	serial->setBaudRate(QSerialPort::Baud57600);
	serial->setDataBits(QSerialPort::Data8);
	serial->setParity(QSerialPort::NoParity);
	serial->setStopBits(QSerialPort::OneStop);
	serial->setFlowControl(QSerialPort::NoFlowControl);

	connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
}

void WaveBuoySerial::readData()
{
	// Checks if the serial port is open, and waits for a full line of data containing 10 values
	if (serial->isOpen() && serial->canReadLine())
	{
		QString line = QString(serial->readLine());
		QStringList lineSplit = line.split(" ", QString::SkipEmptyParts);
		if (lineSplit.size() == 10)
		{
			// Convert integer readings from sensor to real-world values
			sensorValues currentValues;
			currentValues.time = lineSplit.at(0).toDouble()/1000;
			currentValues.GY = -(lineSplit.at(1).toDouble())/65536*1000*3.14/180; // 16-bit, +-500 deg/sec converted to radians
			currentValues.GX = -(lineSplit.at(2).toDouble())/65536*1000*3.14/180;
			currentValues.GZ = -(lineSplit.at(3).toDouble())/65536*1000*3.14/180;
			currentValues.AX = -(lineSplit.at(4).toDouble())/65536*4;			// 12-bit, +-2g
			currentValues.AY = -(lineSplit.at(5).toDouble())/65536*4;
			currentValues.AZ = -(lineSplit.at(6).toDouble())/65536*4;
			currentValues.MX = -(lineSplit.at(7).toDouble())/2048;
			currentValues.MY = -(lineSplit.at(8).toDouble())/2048;
			currentValues.MZ = -(lineSplit.at(9).toDouble())/2048;

			currentValues.calibrate(calib);

			movingAverageBuffer.prepend(currentValues);

			while (movingAverageBuffer.size() > nMovingAverage)
				movingAverageBuffer.pop_back();

			sensorValues movingAverage;
			for (int i = 0; i < movingAverageBuffer.size(); i++)
				movingAverage += movingAverageBuffer[i];

			movingAverage.time = currentValues.time;
			movingAverage /= movingAverageBuffer.size();

			emit(sendSensorUpdate(movingAverage));
		}
	}
}

void WaveBuoySerial::setMovingAverage( int nMovingAverage )
{
	this->nMovingAverage = nMovingAverage;
}

void WaveBuoySerial::disconnectCOM()
{
	delete serial;
	serial = new QSerialPort;
	QStringList portNames;
	foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
		portNames.append(info.portName());
	emit(sendPortInfo(portNames));
}
