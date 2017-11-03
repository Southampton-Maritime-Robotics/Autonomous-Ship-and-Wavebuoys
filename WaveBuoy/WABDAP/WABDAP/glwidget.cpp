#include "glwidget.h"


GLWidget::GLWidget(QWidget *parent): QGLWidget(parent)
{
}

GLWidget::~GLWidget()
{
	deleteTexture(texture);
}

void GLWidget::initializeGL()
{
	initializeGLFunctions();
	qglClearColor(Qt::black);
	initShaders();
	initTextures();

	// Enable depth buffer
	glEnable(GL_DEPTH_TEST);

	// Enable back face culling
	glEnable(GL_CULL_FACE);

	geometries.init();
}

void GLWidget::resizeGL(int w, int h)
{
	// Set OpenGL viewport to cover whole widget
	glViewport(0, 0, w, h);
	// Calculate aspect ratio
	qreal aspect = qreal(w) / qreal(h ? h : 1);
	// Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
	const qreal zNear = 3.0, zFar = 7.0, fov = 45.0;
	// Reset projection
	projection.setToIdentity();
	// Set perspective projection
	projection.perspective(fov, aspect, zNear, zFar);
}

void GLWidget::paintGL()
{
	// Clear color and depth buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// Calculate model view transformation
	QMatrix4x4 matrix;
	matrix.translate(0.0, 0.0, -5.0);
	//rotation = QQuaternion::fromAxisAndAngle(0.535538,0.436676,-0.495649,-0.523162);
// 	QQuaternion zero = QQuaternion(1,1,0,0);
// 	zero.normalize();
//	matrix.rotate(zero);
	matrix.rotate(calib);
	matrix.rotate(rotation);
	// Set modelview-projection matrix
	program.setUniformValue("mvp_matrix", projection * matrix);
	// Use texture unit 0 which contains cube.png
	program.setUniformValue("texture", 0);
	// Draw cube geometry
	geometries.drawCubeGeometry(&program);
}

void GLWidget::initShaders()
{
	// Override system locale until shaders are compiled
	setlocale(LC_NUMERIC, "C");
	// Compile vertex shader
	if (!program.addShaderFromSourceFile(QGLShader::Vertex, ":/vshader.glsl"))
		close();
	// Compile fragment shader
	if (!program.addShaderFromSourceFile(QGLShader::Fragment, ":/fshader.glsl"))
		close();
	// Link shader pipeline
	if (!program.link())
		close();
	// Bind shader pipeline for use
	if (!program.bind())
		close();
	// Restore system locale
	setlocale(LC_ALL, "");
}

void GLWidget::initTextures()
{
	// Load cube.png image
	glEnable(GL_TEXTURE_2D);
	texture = bindTexture(QImage(":/cube.png"));
	// Set nearest filtering mode for texture minification
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	// Set bilinear filtering mode for texture magnification
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// Wrap texture coordinates by repeating
	// f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}

void GLWidget::updateRotation(QQuaternion calib, QQuaternion quart )
{
	this->calib = calib;
	rotation = quart;
	updateGL();
}
