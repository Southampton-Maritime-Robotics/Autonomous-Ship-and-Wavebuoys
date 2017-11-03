#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QtOpenGL/QGLShaderProgram>
#include <QMouseEvent>


#include "geometryengine.h"
class GeometryEngine;

//class QOpenGLFunctions;

class GLWidget : public QGLWidget, protected QGLFunctions
{
	Q_OBJECT

public:
	GLWidget(QWidget *parent);
	~GLWidget();
	void updateRotation(QQuaternion calib, QQuaternion quart);

protected:
	void initializeGL();
	void resizeGL(int w, int h);
  void paintGL();
	void initShaders();
	void initTextures();

private:
	GeometryEngine geometries;
	QGLShaderProgram program;
	GLuint texture;
	QVector2D mousePressPosition;
	QMatrix4x4 projection;
	QVector3D rotationAxis;
	qreal angularSpeed;
	QQuaternion calib;
	QQuaternion rotation;
};

#endif // GLWIDGET_H
