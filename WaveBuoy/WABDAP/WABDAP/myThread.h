#ifndef myThread_h__
#define myThread_h__

#include <QThread>

template <class T>
class Thread
{
public:
	Thread()
	{
		thread = new QThread;
		worker = new T;

		worker->moveToThread(thread);
		QObject::connect(thread, SIGNAL(started()), worker, SLOT(run()));
		QObject::connect(worker, SIGNAL(finished()), thread, SLOT(quit()));
		QObject::connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
		QObject::connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
		thread->start();
	}
	~Thread()
	{
		thread->quit();
		worker->deleteLater();
		thread->deleteLater();
	}
	QThread * getThread()
	{
		return thread;
	}
	T * getWorker()
	{
		return worker;
	}
	bool connect(const QObject * sender, const char * signal, const char * method, Qt::ConnectionType type = Qt::AutoConnection)
	{
		return worker->connect(sender, signal, method, type);
	}
private:
	QThread * thread;
	T * worker;
};

#endif // myThread_h__