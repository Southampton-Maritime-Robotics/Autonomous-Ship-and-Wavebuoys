#include "gui.h"

GUI::GUI(QWidget *parent)
	: QMainWindow(parent)
{

	calibrated = false;
	connected = false;
	hasFilename = false;
	// Setup the UI
	ui.setupUi(this);

	// ... Set tab names and hides calibration tab
	ui.tabWidget->setTabText(0, "3D Output");
	ui.tabWidget->setTabText(1, "Sensor Data");
	ui.tabWidget->setTabText(2, "Settings");
	settingsPage = ui.tabWidget->widget(2);
	ui.tabWidget->removeTab(2);

	// ... Disable start & processing button
	ui.processButton->setDisabled(true);
	ui.startButton->setDisabled(true);
	ui.calStart->setDisabled(true);
	ui.calStop->setDisabled(true);
	ui.calStart2->setDisabled(true);
	ui.calStop2->setDisabled(true);

	//ui.widget->resizeGL(300,300);

	// ... Setup graphs
	setupGraphs();
	movingAverageChanged();

	// Setup slots and signals local to the GUI
	connect(ui.exitButton, SIGNAL(clicked()), this, SLOT(close()));
	connect(ui.serialConnectButton, SIGNAL(clicked()), this, SLOT(connectButtonPressed()));
	connect(ui.movingAverageSpinBox, SIGNAL(valueChanged(int)), this, SLOT(movingAverageChanged()));
	connect(ui.settingsButton, SIGNAL(clicked()), this, SLOT(showCalibrationTab()));
	connect(ui.calLoad, SIGNAL(clicked()), this, SLOT(calLoadButton()));
	connect(ui.calSave, SIGNAL(clicked()), this, SLOT(calSaveButton()));
	connect(ui.calStart, SIGNAL(clicked()), this, SLOT(calStartButton()));
	connect(ui.calStop, SIGNAL(clicked()), this, SLOT(calStopButton()));
	connect(ui.calStart2, SIGNAL(clicked()), this, SLOT(cal2StartButton()));
	connect(ui.calStop2, SIGNAL(clicked()), this, SLOT(cal2StopButton()));
	connect(ui.browseButton, SIGNAL(clicked()), this, SLOT(setFileName()));
	connect(ui.startButton, SIGNAL(clicked()), this, SLOT(startButton()));
	connect(ui.processButton, SIGNAL(clicked()), this, SLOT(processButton()));

	// Setup a timer which triggers the status bar info
	statusBarSerialTimer = new QTimer(this);
	connect(statusBarSerialTimer, SIGNAL(timeout()), this, SLOT(statusBarUpdate()));
	statusBarSerialTimer->start(500);

	ui.calibpic->setPixmap(QPixmap("calibpic.png").scaled(ui.calibpic->size().width(),ui.calibpic->size().height()));
	ui.compasspic->setPixmap(QPixmap("Compass.png").scaledToHeight(ui.compasspic->size().height()));

	setWindowTitle("WABDAP"); // Wave Buoy Directional Analysis Program.
}

GUI::~GUI()
{

}

void GUI::updateSerialComboBox( QStringList portInfo )
{
	ui.serialComboBox->clear();
	foreach (QString text, portInfo)
		ui.serialComboBox->addItem(text);

	ui.serialComboBox->model()->sort(0);
	ui.serialComboBox->setCurrentIndex(0);
}

void GUI::setupGraphs()
{
	ui.gyroPlot->addGraph(); // blue line = X
	ui.gyroPlot->addGraph(); // green line = Y
	ui.gyroPlot->addGraph(); // red line = Z
	ui.gyroPlot->setAutoMargin(false);
	ui.gyroPlot->setMargin(1,1,1,1);
	ui.gyroPlot->xAxis->setAutoTickStep(false);
	ui.gyroPlot->xAxis->setTickStep(1.0);
	ui.gyroPlot->graph(0)->setPen(QPen(Qt::blue));
	ui.gyroPlot->graph(1)->setPen(QPen(Qt::green));
	ui.gyroPlot->graph(2)->setPen(QPen(Qt::red));
	ui.gyroPlot->addGraph(); // blue dot = X
	ui.gyroPlot->addGraph(); // green dot = Y
	ui.gyroPlot->addGraph(); // red dot = Z
	ui.gyroPlot->graph(3)->setPen(QPen(Qt::blue));
	ui.gyroPlot->graph(3)->setLineStyle(QCPGraph::lsNone);
	ui.gyroPlot->graph(3)->setScatterStyle(QCP::ssDisc);
	ui.gyroPlot->graph(4)->setPen(QPen(Qt::green));
	ui.gyroPlot->graph(4)->setLineStyle(QCPGraph::lsNone);
	ui.gyroPlot->graph(4)->setScatterStyle(QCP::ssDisc);
	ui.gyroPlot->graph(5)->setPen(QPen(Qt::red));
	ui.gyroPlot->graph(5)->setLineStyle(QCPGraph::lsNone);
	ui.gyroPlot->graph(5)->setScatterStyle(QCP::ssDisc);
	ui.gyroPlot->setupFullAxesBox();

	ui.accelPlot->addGraph(); // blue line = X
	ui.accelPlot->addGraph(); // green line = Y
	ui.accelPlot->addGraph(); // red line = Z
	ui.accelPlot->setAutoMargin(false);
	ui.accelPlot->setMargin(1,1,1,1);
	ui.accelPlot->xAxis->setAutoTickStep(false);
	ui.accelPlot->xAxis->setTickStep(1.0);
	ui.accelPlot->graph(0)->setPen(QPen(Qt::blue));
	ui.accelPlot->graph(1)->setPen(QPen(Qt::green));
	ui.accelPlot->graph(2)->setPen(QPen(Qt::red));
	ui.accelPlot->addGraph(); // blue dot = X
	ui.accelPlot->addGraph(); // green dot = Y
	ui.accelPlot->addGraph(); // red dot = Z
	ui.accelPlot->graph(3)->setPen(QPen(Qt::blue));
	ui.accelPlot->graph(3)->setLineStyle(QCPGraph::lsNone);
	ui.accelPlot->graph(3)->setScatterStyle(QCP::ssDisc);
	ui.accelPlot->graph(4)->setPen(QPen(Qt::green));
	ui.accelPlot->graph(4)->setLineStyle(QCPGraph::lsNone);
	ui.accelPlot->graph(4)->setScatterStyle(QCP::ssDisc);
	ui.accelPlot->graph(5)->setPen(QPen(Qt::red));
	ui.accelPlot->graph(5)->setLineStyle(QCPGraph::lsNone);
	ui.accelPlot->graph(5)->setScatterStyle(QCP::ssDisc);
	ui.accelPlot->setupFullAxesBox();

	ui.magPlot->addGraph(); // blue line = X
	ui.magPlot->addGraph(); // green line = Y
	ui.magPlot->addGraph(); // red line = Z
	ui.magPlot->setAutoMargin(false);
	ui.magPlot->setMargin(1,1,1,1);
	ui.magPlot->xAxis->setAutoTickStep(false);
	ui.magPlot->xAxis->setTickStep(1.0);
	ui.magPlot->graph(0)->setPen(QPen(Qt::blue));
	ui.magPlot->graph(1)->setPen(QPen(Qt::green));
	ui.magPlot->graph(2)->setPen(QPen(Qt::red));
	ui.magPlot->addGraph(); // blue dot = X
	ui.magPlot->addGraph(); // green dot = Y
	ui.magPlot->addGraph(); // red dot = Z
	ui.magPlot->graph(3)->setPen(QPen(Qt::blue));
	ui.magPlot->graph(3)->setLineStyle(QCPGraph::lsNone);
	ui.magPlot->graph(3)->setScatterStyle(QCP::ssDisc);
	ui.magPlot->graph(4)->setPen(QPen(Qt::green));
	ui.magPlot->graph(4)->setLineStyle(QCPGraph::lsNone);
	ui.magPlot->graph(4)->setScatterStyle(QCP::ssDisc);
	ui.magPlot->graph(5)->setPen(QPen(Qt::red));
	ui.magPlot->graph(5)->setLineStyle(QCPGraph::lsNone);
	ui.magPlot->graph(5)->setScatterStyle(QCP::ssDisc);
	ui.magPlot->setupFullAxesBox();
}

void GUI::receiveSensorUpdate( sensorValues updateValues )
{
	connected = true;
	// this function is called when the main thread propagates new data
	currentValues = updateValues;

	ui.gyroPlot->graph(0)->addData(currentValues.time,currentValues.GX);
	ui.gyroPlot->graph(1)->addData(currentValues.time,currentValues.GY);
	ui.gyroPlot->graph(2)->addData(currentValues.time,currentValues.GZ);
	ui.gyroPlot->graph(3)->clearData();
	ui.gyroPlot->graph(4)->clearData();
	ui.gyroPlot->graph(5)->clearData();
	ui.gyroPlot->graph(3)->addData(currentValues.time,currentValues.GX);
	ui.gyroPlot->graph(4)->addData(currentValues.time,currentValues.GY);
	ui.gyroPlot->graph(5)->addData(currentValues.time,currentValues.GZ);
	double timeRange = (double)ui.timeSlider->value()/10.;
	double gyroRange = (double)ui.gyroSlider->value()*3.1415/180;
	ui.gyroPlot->graph(0)->removeDataBefore(currentValues.time-timeRange<0);
	ui.gyroPlot->xAxis->setRange(currentValues.time+timeRange*0.1, timeRange*1.1, Qt::AlignRight);
	ui.gyroPlot->yAxis->setRange(-gyroRange, gyroRange);
	ui.gyroPlot->replot();

	ui.accelPlot->graph(0)->addData(currentValues.time,currentValues.AX);
	ui.accelPlot->graph(1)->addData(currentValues.time,currentValues.AY);
	ui.accelPlot->graph(2)->addData(currentValues.time,currentValues.AZ);
	ui.accelPlot->graph(3)->clearData();
	ui.accelPlot->graph(4)->clearData();
	ui.accelPlot->graph(5)->clearData();
	ui.accelPlot->graph(3)->addData(currentValues.time,currentValues.AX);
	ui.accelPlot->graph(4)->addData(currentValues.time,currentValues.AY);
	ui.accelPlot->graph(5)->addData(currentValues.time,currentValues.AZ);
	double accelRange = (double)ui.accelSlider->value()/50;
	ui.accelPlot->graph(0)->removeDataBefore(currentValues.time-timeRange<0);
	ui.accelPlot->xAxis->setRange(currentValues.time+timeRange*0.1, timeRange*1.1, Qt::AlignRight);
	ui.accelPlot->yAxis->setRange(-accelRange, accelRange);
	ui.accelPlot->replot();

	ui.magPlot->graph(0)->addData(currentValues.time,currentValues.MX);
	ui.magPlot->graph(1)->addData(currentValues.time,currentValues.MY);
	ui.magPlot->graph(2)->addData(currentValues.time,currentValues.MZ);
	ui.magPlot->graph(3)->clearData();
	ui.magPlot->graph(4)->clearData();
	ui.magPlot->graph(5)->clearData();
	ui.magPlot->graph(3)->addData(currentValues.time,currentValues.MX);
	ui.magPlot->graph(4)->addData(currentValues.time,currentValues.MY);
	ui.magPlot->graph(5)->addData(currentValues.time,currentValues.MZ);
	double magRange = (double)ui.magSlider->value()/100;
	ui.magPlot->graph(0)->removeDataBefore(currentValues.time-timeRange<0);
	ui.magPlot->xAxis->setRange(currentValues.time+timeRange*0.1, timeRange*1.1, Qt::AlignRight);
	ui.magPlot->yAxis->setRange(-magRange, magRange);
	ui.magPlot->replot();
}

void GUI::statusBarUpdate()
{
	if (connected)
	{
		ui.statusBar->showMessage("Serial connected.");
		connected = false;
	}
	else
	{
		ui.statusBar->showMessage("Serial not connected.");
	}
}

void GUI::movingAverageChanged()
{
	emit(sendNMovingAverage( (ui.movingAverageSpinBox->value()>0) ? ui.movingAverageSpinBox->value() : 5 ));
}

void GUI::connectButtonPressed()
{
	emit(connectCOM(ui.serialComboBox->currentText()));
	ui.serialConnectButton->setText("Disconnect");
	disconnect(ui.serialConnectButton, SIGNAL(clicked()), this, SLOT(connectButtonPressed()));
	connect(ui.serialConnectButton, SIGNAL(clicked()), this, SLOT(disconnectButtonPressed()));
	ui.serialComboBox->setDisabled(true);
}

void GUI::disconnectButtonPressed()
{
	emit(disconnectCOM());
	disconnect(ui.serialConnectButton, SIGNAL(clicked()), this, SLOT(disconnectButtonPressed()));
	connect(ui.serialConnectButton, SIGNAL(clicked()), this, SLOT(connectButtonPressed()));
	ui.serialConnectButton->setText("Connect");
	ui.serialComboBox->setDisabled(false);
}
