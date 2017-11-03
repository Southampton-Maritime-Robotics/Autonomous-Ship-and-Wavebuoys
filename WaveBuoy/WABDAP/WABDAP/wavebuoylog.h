#ifndef WAVEBUOYLOG_H
#define WAVEBUOYLOG_H

#include <QObject>
#include <QQuaternion>
#include <QVector>
#include <QFile>
#include <cmath>
#include <QTextStream>
#include "sensorValues.h"
#include <complex>
#include <QDebug>

class WaveBuoyLog : public QObject
{
	Q_OBJECT

public:
	WaveBuoyLog(QObject *parent = 0);
	~WaveBuoyLog();

public slots:
	void run()
	{
		isRecording = false;
	}
	void start(QString fileName)
	{
		this->fileName = fileName;
		dataValues.clear();
		dataOrientation.clear();
		isRecording = true;
	}
	void stop()
	{
		isRecording = false;

		int HIGH_PASS_FILTER = 25;

		// perform wave buoy analysis
		int N = dataValues.size();
		// velocity
		QVector<double> VX;
		QVector<double> VY;
		QVector<double> VZ;
		// displacement
		QVector<double> DX;
		QVector<double> DY;
		QVector<double> DZ;

		double time_step = 0;
		double F  = 0;
		// average frequency
		for (int i = 0; i < N -1; i++)
		{
			time_step += dataValues[i+1].time - dataValues[i].time;
		}
		time_step /= (N-1);
		F = 1./time_step;

		// ... calculate yaw, pitch and roll from Quaternion

		QVector<double> yaw;
		QVector<double> roll;
		QVector<double> pitch;

		for (int i = 0; i < N; i++)
		{
			double q0 = dataOrientation[i].scalar();
			double q1 = dataOrientation[i].x();
			double q2 = dataOrientation[i].y();
			double q3 = dataOrientation[i].z();
			roll.append(atan2(2*(q0*q1 + q2*q3), 1. - 2.*(q1*q1 + q2 * q2)));
			pitch.append(asin(2*(q0*q2 - q3*q1)));
			yaw.append(atan2(2*(q0*q3 + q1*q2), 1. - 2. * (q2*q2 + q3*q3)));
		}

		// ... resolve accelerations to vertical (AZ), longitudinal (AY) and transverse (AX) and convert to m/s/s
		for (int i = 0; i < N; i ++)
		{
			QVector3D accel(dataValues[i].AX, dataValues[i].AY, dataValues[i].AZ);
			QVector3D resolved_accel = dataOrientation[i].rotatedVector(accel);
			dataValues[i].AX = resolved_accel.x() * 9.81;
			dataValues[i].AY = resolved_accel.y() * 9.81;
			dataValues[i].AZ = resolved_accel.z() * 9.81;
		}

		// ... integrate accelerations (converted to m/s/s) once to get velocities
		VX.append(0.);
		VY.append(0.);
		VZ.append(0.);
		for (int i = 1; i < N; i++)
		{
			VX.append((dataValues[i].AX+dataValues[i-1].AX)/2. * time_step + VX[i-1]);
			VY.append((dataValues[i].AY+dataValues[i-1].AY)/2. * time_step + VY[i-1]);
			VZ.append((dataValues[i].AZ+dataValues[i-1].AZ)/2. * time_step + VZ[i-1]);
		}

		// calculate error slope

		QVector<double> m_VX;
		QVector<double> m_VY;
		QVector<double> m_VZ;
		m_VX.resize(VX.size()); m_VX.fill(0.0);
		m_VY.resize(VY.size()); m_VY.fill(0.0);
		m_VZ.resize(VZ.size()); m_VZ.fill(0.0);

		for (int i = 1; i < N; i++)
		{
			double m_vx = 0;
			double m_vy = 0;
			double m_vz = 0;
			int count= 0;
			for (int j = -HIGH_PASS_FILTER; j < 0; j++)
			{
				if ( i + j >= 0 && i + j < N)
				{
					count++;
					m_vx += VX[i+j];
					m_vy += VY[i+j];
					m_vz += VZ[i+j];
				}
			}
			if (count != 0)
			{
				m_vx /= count;
				m_vy /= count;
				m_vz /= count;
				m_VX[i] = VX[i] - m_vx;
				m_VY[i] = VY[i] - m_vy;
				m_VZ[i] = VZ[i] - m_vz;
			}
		}
		for (int i = 0; i < N; i++)
		{
			VX[i] = m_VX[i];
			VY[i] = m_VY[i];
			VZ[i] = m_VZ[i];
		}

		// ... integrate velocities to get displacements
		DX.append(0.);
		DY.append(0.);
		DZ.append(0.);
		// possibly reset displacement if accel is zero (but not a good idea)
		for (int i = 1; i < N; i++)
		{
			DX.append((VX[i]+VX[i-1])/2. * time_step + DX[i-1]);
			DY.append((VY[i]+VY[i-1])/2. * time_step + DY[i-1]);
			DZ.append((VZ[i]+VZ[i-1])/2. * time_step + DZ[i-1]);
		}

		// calculate error slope

		QVector<double> m_DX;
		QVector<double> m_DY;
		QVector<double> m_DZ;
		m_DX.resize(DX.size()); m_DX.fill(0.0);
		m_DY.resize(DY.size()); m_DY.fill(0.0);
		m_DZ.resize(DZ.size()); m_DZ.fill(0.0);

		for (int i = 1; i < N; i++)
		{
			double m_dx = 0;
			double m_dy = 0;
			double m_dz = 0;
			int count= 0;
			for (int j = -HIGH_PASS_FILTER; j < 0; j++)
			{
				if ( i + j >= 0 && i + j < N)
				{
					count++;
					m_dx += DX[i+j];
					m_dy += DY[i+j];
					m_dz += DZ[i+j];
				}
			}
			if (count != 0)
			{
				m_dx /= count;
				m_dy /= count;
				m_dz /= count;
				m_DX[i] = DX[i] - m_dx;
				m_DY[i] = DY[i] - m_dy;
				m_DZ[i] = DZ[i] - m_dz;
			}
		}
		for (int i = 0; i < N; i++)
		{
			DX[i] = m_DX[i];
			DY[i] = m_DY[i];
			DZ[i] = m_DZ[i];
		}

		// ... chop off the first 250 values, this gives time for the high-pass filter to settle
		dataValues.remove(0, HIGH_PASS_FILTER * 3);
		dataOrientation.remove(0,HIGH_PASS_FILTER * 3);
		VX.remove(0, HIGH_PASS_FILTER * 3);
		VY.remove(0, HIGH_PASS_FILTER * 3);
		VZ.remove(0, HIGH_PASS_FILTER * 3);
		DX.remove(0, HIGH_PASS_FILTER * 3);
		DY.remove(0, HIGH_PASS_FILTER * 3);
		DZ.remove(0, HIGH_PASS_FILTER * 3);
		roll.remove(0, HIGH_PASS_FILTER * 3);
		pitch.remove(0, HIGH_PASS_FILTER * 3);
		yaw.remove(0, HIGH_PASS_FILTER * 3);
		
		N = N - HIGH_PASS_FILTER * 3;

		// calculate the average velocities and displacements and subtract from the values
		// note that the re-averaging of velocities does not have to occur before the 2nd integration (for displacement)
		// in fact, the velocities are not used any further by the algorithm, but corrected for completeness
		// ... calculate average velocity and subtract from velocities
		double VX_bar  = 0, VY_bar = 0, VZ_bar = 0;
		for (int i = 0; i < N; i++)
		{
			VX_bar += VX[i];
			VY_bar += VY[i];
			VZ_bar += VZ[i];
		}
		VX_bar /= N; VY_bar /= N; VZ_bar /= N;

		for (int i = 0; i < N; i++)
		{
			VX[i] -= VX_bar;
			VY[i] -= VY_bar;
			VZ[i] -= VZ_bar;
 		}
		double DX_bar  = 0, DY_bar = 0, DZ_bar = 0;
		for (int i = 0; i < N; i++)
		{
			DX_bar += DX[i];
			DY_bar += DY[i];
			DZ_bar += DZ[i];
		}
		DX_bar /= N; DY_bar /= N; DZ_bar /= N;

		for (int i = 0; i < N; i++)
		{
			DX[i] -= DX_bar;
			DY[i] -= DY_bar;
			DZ[i] -= DZ_bar;
		}

		// recalculate average frequency and timestep of data
		for (int i = 0; i < N -1; i++)
		{
			time_step += dataValues[i+1].time - dataValues[i].time;
		}
		time_step /= (N-1);
		F = 1./time_step;


		// ... perform fast fourier transforms
		QVector<double> FFT_Freq;
		QVector<std::complex<double>> C1;
		QVector<std::complex<double>> C2;
		QVector<std::complex<double>> C3;
		QVector<double> C11;
		QVector<double> C22;
		QVector<double> C33;
		QVector<double> C23;
		QVector<double> Q12;
		QVector<double> Q13;
		double totalTime = (dataValues[N-1].time - dataValues[0].time);

		// 1 - heave
		for(int k = 0; k < N; k++)
		{
			std::complex<double> sum(0.0,0.0);
			for(int j = 0; j < N; j++)
			{
				int integers = -2*j*k;
				std::complex<double> exponent(0.0, 3.1415/N*(double)integers);
				sum += DZ[j] * std::exp(exponent);
			}
			if (k < N/2)
			{
				FFT_Freq.append(k / totalTime);
				C1.append(sum);
			}
		}
		// 2 - roll
		for(int k = 0; k < N; k++)
		{
			std::complex<double> sum(0.0,0.0);
			for(int j = 0; j < N; j++)
			{
				int integers = -2*j*k;
				std::complex<double> exponent(0.0, 3.1415/N*(double)integers);
				sum += roll[j] * std::exp(exponent);
			}
			if (k < N/2)
				C2.append(sum);
		}
		// 3 - pitch
		for(int k = 0; k < N; k++)
		{
			std::complex<double> sum(0.0,0.0);
			for(int j = 0; j < N; j++)
			{
				int integers = -2*j*k;
				std::complex<double> exponent(0.0, 3.1415/N*(double)integers);
				sum += pitch[j] * std::exp(exponent);
			}
			if (k < N/2)
				C3.append(sum);
		}

		// C11
		for (int i = 0; i < N/2; i++)
		{
			C11.append(abs(C1[i])*abs(C1[i]) / (2.*(FFT_Freq[1])) * (2./N)*(2./N) );
			qDebug() << C11[i] << endl;
		}
		// C22
		for (int i = 0; i < N/2; i++)
			C22.append(abs(C2[i])*abs(C2[i]) / (2.*(FFT_Freq[1])) * (2./N)*(2./N) );
		// C33
		for (int i = 0; i < N/2; i++)
			C33.append(abs(C3[i])*abs(C3[i]) / (2.*(FFT_Freq[1])) * (2./N)*(2./N) );
		// C23
		for (int i = 0; i < N/2; i++)
			C23.append( (real(C2[i])*real(C3[i]) + imag(C2[i]) * imag(C3[i])) / (2.*(FFT_Freq[1])) * (2./N)*(2./N)  );
		// Q12
		for (int i = 0; i < N/2; i++)
			Q12.append( (imag(C1[i])*real(C2[i]) - real(C1[i]) * imag(C2[i])) / (2.*(FFT_Freq[1])) * (2./N)*(2./N)  );
		// Q13
		for (int i = 0; i < N/2; i++)
			Q13.append( (imag(C1[i])*real(C3[i]) - real(C1[i]) * imag(C3[i])) / (2.*(FFT_Freq[1])) * (2./N)*(2./N)  );

	
		// Calculate non-directional wave spectra
		double m0 = 0, m1 = 0, m2 = 0, m3 = 0, m4 = 0;

		for (int i = 0; i < N/2 - 1; i++)
		{
			m0 += (C11[i]+C11[i+1])/2 * (FFT_Freq[i+1] - FFT_Freq[i]);
			m1 += (C11[i]+C11[i+1])/2 * (FFT_Freq[i+1] - FFT_Freq[i]) * FFT_Freq[i] ;
			m2 += (C11[i]+C11[i+1])/2 * (FFT_Freq[i+1] - FFT_Freq[i]) * FFT_Freq[i] * FFT_Freq[i];
			m3 += (C11[i]+C11[i+1])/2 * (FFT_Freq[i+1] - FFT_Freq[i]) * FFT_Freq[i] * FFT_Freq[i] * FFT_Freq[i] ;
			m4 += (C11[i]+C11[i+1])/2 * (FFT_Freq[i+1] - FFT_Freq[i]) * FFT_Freq[i] * FFT_Freq[i] * FFT_Freq[i] * FFT_Freq[i] ;

		}
		
		double mean_frequency = m1/m0;
		double mean_period = m0/m1;
		double mean_zero_crossing_period = sqrt(m0/m2);
		double _e = sqrt(1. - (m2*m2/m0/m4));
		double sig_wave_height = 4. * sqrt(m0) * sqrt(1 - _e*_e/2.);
		double sig_wave_amplitude = sig_wave_height/2.;

		QVector<double> k;
		QVector<double> a0;
		QVector<double> a1;
		QVector<double> a2;
		QVector<double> b1;
		QVector<double> b2;
		QVector<double> r1;
		QVector<double> r2;
		QVector<double> theta1;
		QVector<double> theta2;

		for (int i = 0; i < N/2; i++)
		{
			k.append(sqrt((C22[i]+C33[i]) / C11[i]));
			a0.append(C11[i]/3.1415);
			a1.append(Q12[i]/(k[i] * 3.1415));
			a2.append((C22[i]- C33[i])/(k[i]*k[i]*3.1415));
			b1.append(Q13[i]/(k[i]*3.1415));
			b2.append(2*C23[i]/(k[i]*k[i]*3.1415));
			r1.append(1/a0[i] * sqrt(a1[i]*a1[i] + b1[i]*b1[i]));
			r2.append(1/a0[i] * sqrt(a2[i]*a2[i] + b2[i]*b2[i]));
			theta1.append(atan2(b1[i],a1[i]));
			theta2.append(.5*atan2(b2[i],a2[i]));
		}


		QVector<QVector<double>> S; // directional spectrum
		for (int i = 0; i < 360; i++)
		{
			QVector<double> S_angle;
			for (int j = 0; j < N/2; j++)
			{
				double theta = 2.*3.1415 * (double)i/360.;
				S_angle.append(C11[j]/3.1415 * (1./2. + /*2/3 **/ r1[j] * cos(theta - theta1[j]) + /*1/6 **/ r2[j] * cos(2 * (theta - theta2[j]))));
			}
			S.append(S_angle);
		}

		saveFile = new QFile(fileName);
		saveFile->open(QIODevice::WriteOnly);
		QTextStream output(saveFile);

		output << "################################################" << endl
					 << "#   RESULTS FROM WAVAB" << endl
					 << "################################################" << endl;

		output << "Sig Wave Height, " << sig_wave_height << endl;
		output << "Sig Wave Amplitude, " << sig_wave_amplitude << endl;
		output << "Mean Frequency, " << mean_frequency << endl;
		output << "Mean Period, " << mean_period << endl;
		output << "Mean Zero Crossing Period, " << mean_zero_crossing_period << endl
					 << "################################################" << endl;
		output << "Time, AX, AY, AZ, VX, VY, VZ, DX, DY, DZ, Yaw, Pitch, Roll, q0, q1, q2, q3, FFT_Freq, C11, C22, C33, C23, Q12, Q13" << endl
					 << "sec, (m/s/s), (m/s/s), (m/s/s), (m/s), (m/s), (m/s), m, m, m, rad, rad, rad, , , , ,Hz, , , , , , , " << endl;

		for (int i = 0; i < N; i++)
		{
			output << dataValues[i].time << "," << dataValues[i].AX << "," << dataValues[i].AY << "," << dataValues[i].AZ << ",";
			output << VX[i] << "," << VY[i] << "," << VZ[i] << ",";
			output << DX[i] <<"," << DY[i]<<"," << DZ[i]<< ",";
			output << yaw[i]<< "," << pitch[i]<< "," << roll[i]<< "," << dataOrientation[i].scalar() << "," << dataOrientation[i].x() << "," << dataOrientation[i].y() << "," << dataOrientation[i].z() << ",";
			if (i < N/2)
				output << FFT_Freq[i] << "," << C11[i] << "," << C22[i] << "," << C33[i] << "," << C23[i] << "," << Q12[i] << "," << Q13[i] << ",";
			else
				output << "," << "," << "," << "," << "," << ",";
			output << endl;
		}


		saveFile->close();

		delete saveFile;
		QString ThreeDFilename = fileName;
		ThreeDFilename.chop(4);
		ThreeDFilename += "_3D.csv";
		saveFile = new QFile(ThreeDFilename);
		saveFile->open(QIODevice::WriteOnly);
		QTextStream output3D(saveFile);

		output3D << "#Angle, #FFT_Freq, #Spectra" << endl;

		for (int i = 0; i < 360; i++)
			for (int j = 0; j < N/2; j++)
			{
				output3D << (double)i/360.*2.*3.1415 << "," << FFT_Freq[j] << "," << S[i][j] << endl;
			}
		
		saveFile->close();
		delete saveFile;

	}




	void receiveSensorUpdate(sensorValues updateValues, QQuaternion updateOrientation)
	{
		if (isRecording)
		{
			dataValues.append(updateValues);
			dataOrientation.append(updateOrientation);
		}
	}

signals:
	void finished();

private:
	QString fileName;
	bool isRecording;
	QFile * saveFile;
	QVector<sensorValues> dataValues;
	QVector<QQuaternion> dataOrientation;
};

#endif // WAVEBUOYLOG_H
