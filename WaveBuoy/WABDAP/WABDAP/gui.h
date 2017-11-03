#ifndef GUI_H
#define GUI_H

#include <QtWidgets/QMainWindow>
#include <QWidget>
#include <QStringList>
#include <QTimer>
#include <QStatusBar>
#include <QQuaternion>
#include <QFileDialog>
#include <QMessageBox>
#include "ui_gui.h"
#include "sensorValues.h"

class GUI : public QMainWindow
{
	Q_OBJECT

public:
	GUI(QWidget *parent = 0);
	~GUI();

signals:
	void connectCOM(QString portName);
	void disconnectCOM();
	void sendNMovingAverage(int nMovingAverage);
	void loadCal(QString fileName);
	void saveCal(QString fileName);
	void calStartSignal();
	void calStopSignal();
	void calStartSignal2();
	void calStopSignal2();
	void startLogger(QString);
	void stopLogger();
public slots:
	void calibDone()
	{
		settingsPage = ui.tabWidget->widget(2);
		ui.tabWidget->removeTab(2);
		calibrated = true;
	}
	void setFileName()
	{
		ui.saveEdit->setText(QFileDialog::getSaveFileName(this, tr("Save As..."), 0, tr("*.csv")) + ".csv");
		ui.startButton->setDisabled(false);
	}
	void startButton()
	{
		if (!calibrated)
		{
			QMessageBox msgBox;
			msgBox.setText("The sensor has not been calibrated.");
			msgBox.setInformativeText("Run anyway?");
			msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
			msgBox.setDefaultButton(QMessageBox::Cancel);
			int ret = msgBox.exec();
			switch (ret) {
			case QMessageBox::Ok:
				break;
			default:
				return;
			}
		}
		if (!connected)
		{
			QMessageBox msgBox;
			msgBox.setText("The sensor is not connected");
			//msgBox.setInformativeText("Run anyway?");
			msgBox.setStandardButtons(QMessageBox::Ok);
			msgBox.setDefaultButton(QMessageBox::Ok);
			msgBox.exec();
			return;
		}
		emit(startLogger(ui.saveEdit->text()));
		ui.processButton->setDisabled(false);
	}
	void processButton()
	{
		emit(stopLogger());
	}
	void updateSerialComboBox(QStringList portInfo);
	void connectButtonPressed();
	void movingAverageChanged();
	void statusBarUpdate();
	void setupGraphs();
	void receiveSensorUpdate(sensorValues updateValues);
	void receiveQuaternionUpdate(QQuaternion updateQ)
	{
		ui.widget->updateRotation(calValues.ref, updateQ);
	}
	void disconnectButtonPressed();
	void showCalibrationTab()
	{
		ui.tabWidget->addTab(settingsPage, "Calibration");
		ui.tabWidget->widget(0)->hide();
		ui.tabWidget->widget(1)->hide();
		ui.tabWidget->setCurrentIndex(2);
	}
	void calLoadButton()
	{
		ui.calLoadEdit->setText(QFileDialog::getOpenFileName(this, tr("Load calibration file"), 0,  tr("*.cal")));
		emit(loadCal(ui.calLoadEdit->text()));
		ui.calLoadEdit->setDisabled(true);
		ui.calSaveEdit->setDisabled(true);
		ui.calLoad->setDisabled(true);
		ui.calSave->setDisabled(true);
	}
	void calSaveButton()
	{
		ui.calSaveEdit->setText(QFileDialog::getSaveFileName(this, tr("Save As..."), 0,  tr("*.cal")) + ".cal");
		emit(saveCal(ui.calSaveEdit->text()));
		ui.calLoadEdit->setDisabled(true);
		ui.calSaveEdit->setDisabled(true);
		ui.calLoad->setDisabled(true);
		ui.calSave->setDisabled(true);
		ui.calStart->setDisabled(false);
	}
	void calStartButton()
	{
		emit(calStartSignal());
		ui.calStart->setDisabled(true);
		ui.calStop->setDisabled(false);
	}
	void calStopButton()
	{
		ui.calStop->setDisabled(true);
		emit(calStopSignal());
		ui.calStart2->setDisabled(false);
	}
	void cal2StartButton()
	{
		emit(calStartSignal2());
		ui.calStart2->setDisabled(true);
		ui.calStop2->setDisabled(false);
	}
	void cal2StopButton()
	{
		ui.calStop2->setDisabled(true);
		emit(calStopSignal2());
	}
	void receiveCalibration(calibrationValues cal)
	{
		calValues = cal;
	}

private:
	bool connected;
	bool calibrated;
	bool hasFilename;
	sensorValues currentValues;
	calibrationValues calValues;
	Ui::GUIClass ui;
	QWidget * settingsPage;
	QTimer * statusBarSerialTimer;
};

#endif // GUI_H
