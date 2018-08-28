#ifndef RENDERINGWIDGET_H
#define RENDERINGWIDGET_H
//////////////////////////////////////////////////////////////////////////
#include <QOpenGLWidget>
#include "Processor.h"
class Meshprint;
class MainWindow;
class CArcBall;
class RenderingWidget : public QOpenGLWidget
{
	Q_OBJECT
public:
	Meshprint* parent;
	RenderingWidget(QWidget *parent, MainWindow* mainwindow=0);
	~RenderingWidget();

private:
	Processor procesoor;
protected:
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();
	void timerEvent(QTimerEvent *e);

	// mouse events
	void mousePressEvent(QMouseEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);
	void mouseDoubleClickEvent(QMouseEvent *e);
	void wheelEvent(QWheelEvent *e);

public:
	void keyPressEvent(QKeyEvent *e);
	void keyReleaseEvent(QKeyEvent *e);

signals:
	void sendMsgtoDialog(QString);

private:
	void Render();
	void SetLight();
public slots:
	void SetBackground();
	void ReadMesh();
	void WriteMesh();
	void CheckDrawPoint();
	void CheckDrawEdge();
	void CheckDrawFace();
	void CheckLight();
	void CheckDrawTexture();
	void CheckDrawAxes();
	void DoSliceAndHatch();

	void AddSupportStructure();

private:
	void DrawAxes(bool bv);
	void DrawPoints(bool);
	void DrawEdge(bool);
	void DrawFace(bool);
	void DrawTexture(bool);
	void DrawGrid(bool bV);
	void DrawSlice(bool bv);
	void DrawHatch(bool bv);

public:
	MainWindow					*ptr_mainwindow_;
	CArcBall					*ptr_arcball_;
	// Texture
	GLuint						texture_[1];
	bool						is_load_texture_;
	// eye
	GLfloat						eye_distance_{5.0};
	GLfloat						eye_goal_[3];
	GLfloat						eye_direction_[3];
	QPoint						current_position_;
	// Render information
	bool						is_draw_point_;
	bool						is_draw_edge_;
	bool						is_draw_face_;
	bool						is_draw_texture_;
	bool						has_lighting_;
	bool						is_draw_axes_;
};

#endif // RENDERINGWIDGET_H
