#include "gui.h"
#include "wavebuoyserial.h"
#include "wavebuoyimu.h"
#include "myThread.h"
#include "sensorValues.h"
#include "wavebuoycal.h"
#include "wavebuoylog.h"
#include "qMessageHandler.h"
#include <QObject>
#include <QtDebug>
#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <QQuaternion>
#include <QtWidgets/QApplication>

// to do, reset arrays in repeatable functions
// data cleanup in destructors

int main(int argc, char *argv[])
{
	qInstallMessageHandler(customMessageHandler);

	QApplication a(argc, argv);

	GUI frontend;//(0, "Hello", 0);
	Thread<WaveBuoySerial> backendWBSerial;
	Thread<WaveBuoyIMU> backendWBIMU;
	Thread<WaveBuoyCal> backendWBCal;
	Thread<WaveBuoyLog> backendWBLog;
	

	qRegisterMetaType<sensorValues>("sensorValues");
	qRegisterMetaType<calibrationValues>("calibrationValues");
	qRegisterMetaType<int>("int");

	// Connect serial backend thread to IMU calculator backend thread
	backendWBIMU.connect		( backendWBSerial.getWorker(),SIGNAL(sendSensorUpdate(sensorValues)),	SLOT(receiveSensorUpdate(sensorValues)));
	// Connect serial backend thread to calibration backend thread
	backendWBCal.connect		( backendWBSerial.getWorker(),SIGNAL(sendSensorUpdate(sensorValues)),	SLOT(receiveSensorUpdate(sensorValues)));
	backendWBSerial.connect ( backendWBCal.getWorker(),		SIGNAL(sendCalibration(calibrationValues)), SLOT(receiveCalibration(calibrationValues)));
	// Connect serial backend to GUI
	frontend.connect				( backendWBSerial.getWorker(),SIGNAL(sendPortInfo(QStringList)),			SLOT(updateSerialComboBox(QStringList)));
	backendWBSerial.connect	(&frontend,										SIGNAL(connectCOM(QString)),						SLOT(connectCOM(QString)));
	backendWBSerial.connect	(&frontend,										SIGNAL(disconnectCOM()),								SLOT(disconnectCOM()));
	frontend.connect				( backendWBSerial.getWorker(),SIGNAL(sendSensorUpdate(sensorValues)) ,SLOT(receiveSensorUpdate(sensorValues)));
	backendWBSerial.connect	(&frontend,										SIGNAL(sendNMovingAverage(int)),				SLOT(setMovingAverage(int)));
	// connect IMU backened to GUI
	frontend.connect				(backendWBIMU.getWorker(),		SIGNAL(sendQuaternion(QQuaternion)),		SLOT(receiveQuaternionUpdate(QQuaternion)));
	// connect Calibration backend to GUI
	backendWBCal.connect		(&frontend,										SIGNAL(loadCal(QString)),								SLOT(loadCal(QString)));
	backendWBCal.connect		(&frontend,										SIGNAL(saveCal(QString)),								SLOT(saveCal(QString)));
	backendWBCal.connect		(&frontend,										SIGNAL(calStartSignal()),								SLOT(begin()));
	backendWBCal.connect		(&frontend,										SIGNAL(calStopSignal()),								SLOT(end()));
	backendWBCal.connect		(&frontend,										SIGNAL(calStartSignal2()),							SLOT(begin2()));
	backendWBCal.connect		(&frontend,										SIGNAL(calStopSignal2()),								SLOT(end2()));
	frontend.connect				( backendWBCal.getWorker(),		SIGNAL(sendCalibration(calibrationValues)), SLOT(receiveCalibration(calibrationValues)));
	frontend.connect				( backendWBCal.getWorker(),		SIGNAL(calibDone()),										SLOT(calibDone()));
	// connect IMU to logger
	backendWBLog.connect		( backendWBIMU.getWorker(),		SIGNAL(sendLogger(sensorValues, QQuaternion)), SLOT(receiveSensorUpdate(sensorValues, QQuaternion)));
	// connect logger to GUI
	backendWBLog.connect		(&frontend,										SIGNAL(startLogger(QString)),						SLOT(start(QString)));
	backendWBLog.connect		(&frontend,										SIGNAL(stopLogger()),						SLOT(stop()));

	void loadCal(QString fileName);
	void saveCal(QString fileName);
	void calStartSignal();
	void calStopSignal();

	frontend.show();
	
	return a.exec();
}
