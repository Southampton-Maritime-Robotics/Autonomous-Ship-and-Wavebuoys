#ifndef WAVEBUOYCAL_H
#define WAVEBUOYCAL_H

#include <QObject>
#include <QFile>
#include "sensorValues.h"
#include <QVector>
#include <QDataStream>
#include <stdio.h>
#include <math.h>
#include <QDebug>
#include <malloc.h>
#include <string.h>
#include <float.h>
#include <QtGlobal>

class WaveBuoyCal : public QObject
{
	Q_OBJECT

public:
	WaveBuoyCal(QObject *parent = 0);
	~WaveBuoyCal();
public slots:
	void run();
	void receiveSensorUpdate( sensorValues updateValues );
	void loadCal(QString fileName);
	void saveCal(QString fileName);
	void begin();
	void end();
	void begin2()
	{
		calib2Running = true;
	}
	void end2()
	{
		calib2Running = false;
		sensorValues average;
		// get average Gyro and Accel
		for (int i = 0; i < calibrationArray2.size(); i++)
		{
			average += calibrationArray2[i];
		}
		average /= calibrationArray2.size();

		calib.GX = average.GX;
		calib.GY = average.GY;
		calib.GZ = average.GZ;

		qDebug() << " CALIBRATION DATA SAVED:" << endl;
		qDebug() << "A0\tA1\tA2\tA3\tA4\tA5\tA6\tA7\tA8\tBX\tBY\tBZ"  << endl;
		qDebug() << calib.A0 << "\t" << calib.A1 << "\t" << calib.A2 << "\t"  <<
								calib.A3 << "\t" << calib.A4 << "\t" << calib.A5 << "\t"  <<
								calib.A6 << "\t" << calib.A7 << "\t" << calib.A8 << "\t"  <<
								calib.BX << "\t" << calib.BY << "\t" << calib.BZ << endl;
		qDebug() << "GX_0\tGY_0\tGZ_0" << endl;
		qDebug() << calib.GX << "\t" << calib.GY << "\t" << calib.GZ << endl;
		qDebug() << "q0\tq1\tq2\tq3" << endl;
		qDebug() << calib.ref.scalar() << "\t" << calib.ref.x() << "\t" << calib.ref.y() << "\t" << calib.ref.z() << endl;

		QDataStream outDS(inOrOut);
		outDS <<	calib.A0 << calib.A1 << calib.A2 <<
							calib.A3 << calib.A4 << calib.A5 <<
							calib.A6 << calib.A7 << calib.A8 <<
							calib.BX << calib.BY << calib.BZ <<
							calib.GX << calib.GY << calib.GZ <<
							calib.ref.scalar() << calib.ref.x() <<
							calib.ref.y() << calib.ref.z();
		inOrOut->close();
		delete inOrOut;

		emit(sendCalibration(calib));
		emit(calibDone());
	}

signals:
	void sendCalibration(calibrationValues updateCalibration);
	void finished();
	void calibDone();

private:
	QVector<sensorValues> calibrationArray;
	QVector<sensorValues> calibrationArray2;
	QFile * inOrOut;
	bool calibRunning;
	bool calib2Running;
	calibrationValues calib;
	sensorValues currentValues;
	double q0, q1, q2, q3;
	bool firstIteration;

	void Multiply_Self_Transpose(double *C, double *A, int nrows, int ncols);
	void Get_Submatrix(double *S, int mrows, int mcols, double *A, int ncols, int row, int col);
	int Choleski_LU_Decomposition(double *A, int n);
	int Choleski_LU_Solve(double *LU, double B[], double x[], int n);
	int Choleski_LU_Inverse(double *LU, int n);
	void Multiply_Matrices(double *C, double *A, int nrows, int ncols, double *B, int mcols);
	static void Identity_Matrix(double *A, int n);
	int Hessenberg_Form_Elementary(double *A, double* S, int n);
	static void Hessenberg_Elementary_Transform(double *H, double* S, int perm[], int n);
	void Copy_Vector(double *d, double *s, int n);
	int QR_Hessenberg_Matrix( double *H, double *S, double eigen_real[], double eigen_imag[], int n, int max_iteration_count);
	static void One_Real_Eigenvalue(double Hrow[], double eigen_real[], double eigen_imag[], int row, double shift);
	static void Two_Eigenvalues(double *H, double* S, double eigen_real[], double eigen_imag[], int n, int row, double shift);
	static void Update_Row(double *Hrow, double cos, double sin, int n, int row);
	static void Update_Column(double* H, double cos, double sin, int n, int col);
	static void Update_Transformation(double *S, double cos, double sin, int n, int k);
	static void Double_QR_Iteration(double *H, double *S, int min_row, int max_row, int n, double* shift, int iteration);
	static void Product_and_Sum_of_Shifts(double *H, int n, int max_row, double* shift, double *trace, double *det, int iteration);;
	static int Two_Consecutive_Small_Subdiagonal(double* H, int min_row, int max_row, int n, double trace, double det);;
	static void Double_QR_Step(double *H, int min_row, int max_row, int min_col, double trace, double det, double *S, int n); 
	static void BackSubstitution(double *H, double eigen_real[], double eigen_imag[], int n);
	static void BackSubstitute_Real_Vector(double *H, double eigen_real[], double eigen_imag[], int row,  double zero_tolerance, int n);
	static void BackSubstitute_Complex_Vector(double *H, double eigen_real[], double eigen_imag[], int row,  double zero_tolerance, int n);
	static void Calculate_Eigenvectors(double *H, double *S, double eigen_real[], double eigen_imag[], int n);
	static void Complex_Division(double x, double y, double u, double v, double* a, double* b);
	void Transpose_Square_Matrix( double *A, int n );
	int Lower_Triangular_Inverse(double *L, int n);
	int Lower_Triangular_Solve(double *L, double B[], double x[], int n);
	int Upper_Triangular_Solve(double *U, double B[], double x[], int n);
	int Upper_Triangular_Inverse(double *U, int n);
	void Interchange_Rows(double *A, int row1, int row2, int ncols);
	void Interchange_Columns(double *A, int col1, int col2, int nrows, int ncols);
	void calibrationOrientation( sensorValues calibValues, QQuaternion & ref );
	double invSqrt( double x );
};

#endif // WAVEBUOYCAL_H
