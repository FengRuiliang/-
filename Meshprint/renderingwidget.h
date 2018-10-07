#ifndef RENDERINGWIDGET_H
#define RENDERINGWIDGET_H
//////////////////////////////////////////////////////////////////////////
#include <QOpenGLWidget>
#include "globalFunctions.h"
#include <QEvent>
#include "HE_mesh/Vec.h"
#include "Hatch.h"
#include "AMObject.h"
#include "Supportbyslice.h"
using trimesh::vec;
using trimesh::point;
typedef trimesh::vec3  Vec3f;
class MainWindow;

class RenderingWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	Meshprint* parent;

	RenderingWidget(QWidget *parent, MainWindow* mainwindow=0);
	~RenderingWidget();

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
	//void meshInfo(int, int, int);
	//void operatorInfo(QString);
	void sendMsgtoDialog(QString);

private:
	void Render();
	void SetLight();
	bool is_draw_region_;
	bool is_draw_support_;
	public slots:
	void ResetView();
	void RecvMsg(QString);
	void ApplyMaintenance();

	void SetBackground();
	void ReadSingleMesh();
	void ReadMesh();
	void WriteMesh();
	void SaveContour();
	void Export();
	void LoadTexture();
	void SetSliceCheckId(int id);
	void CheckDrawPoint();
	void CheckDrawEdge();
	void CheckDrawFace();
	void CheckLight();
	void CheckGrid();
	void CheckDrawTexture();
	void CheckDrawAxes();
	void CheckDrawCutPieces();
	void Checkmoduletranslate();
	void CheckSetFace();

	void CheckRegion(bool bv);
	
	void CheckSupport(bool bv);
	void CheckRotateModel(bool bv);
	// support operators
	void AddPointSupport();
	void AddLineSupport();
	void DeleteSupport();
	void AutoSupport();
	

	void setPointD(double diameter);
	void setPointH(double diameter);
	void setLineD(double diameter);
	void setLineH(double diameter);
	
	void setThreshold(double threshold) { THRESHOLD = cos(3.1415926 * threshold / 180); };
	void setGap(double gap) { GAP = gap; };
	void setSeglength(double length) { SEGLENGTH = length; };
	void setReso(double reso) { RESO = reso; };
	void setVerticalgap(double verticalgap) { VERTICALGAP = verticalgap; };
	void setfieldWidth(double width);
	void setfieldHeight(double height);
	void setlineOverlap(int lineoverlap);
	void setfieldOverlap(double fieldoverlap);
	void setHatchDis(double value);
	void setThickness(double thick);
	void setAngle(int angle);
	void SetAllHatch(bool bv) { is_show_all = bv; }
	void FindRegion();
	void setFildID(int id) {
		fildID = id; 
		update();
	};
	
	//hatch operator
	void setHatchType(int type_) {
	for (int i=0;i<ctn_obj.size();i++)
	{
		ctn_obj[i]->hatch_type_ =type_;
	}
	};
	void setLaserPower(int power) { laser_power_hatch_ = power; };
	void setLaserSpeed(int speed) { laser_speed_hatch_ = speed; };
	void setLaserPowerPolygon(int power) { laser_power_polygon_ = power; };
	void setLaserSpeedPolygon(int speed) { laser_speed_polygon_ = speed; };
	void setOffset(double dis)
	{
		offset_dis_ = dis; 
	};

	// SLICE operators
	void cutinPieces();
	void cutinPiecesSup();
	void renderdoHatch();

	void objectTransformation(float * matrix);
	void Translation();
	void SetDirection(int id);
	void setThickness(float thickness){thickness_ = thickness;};
	public slots:
	void SelectFace(int x, int y);

private:
	void DrawAxes(bool bv);
	void DrawPoints(bool);
	void DrawEdge(bool);
	void DrawFace(bool);
	void DrawSupport(bool bv);
	void DrawSupFace(bool bv);
	void DrawGrid(bool bV);
	void DrawCutPieces(bool bv);
	void DrawCutPiecesSup(bool bv);
	void DrawHatch(bool bv);
	void DrawHatchsup(bool bv);
public:
	MainWindow					*ptr_mainwindow_;
	CArcBall					*ptr_arcball_;
	std::vector<AMObject*>		ctn_obj;
	GLuint						texture_[1];
	bool						is_load_texture_;
	bool						isAddPoint;
	bool						isAddLine;
	bool						isDelete;

	std::vector<int>			delete_points_;
	std::vector<Vec3f>			line_points_;

	// eye
	GLfloat						eye_distance_;
	point						eye_goal_;
	vec							eye_direction_;
	QPoint						current_position_;

	// Render information
	bool						is_draw_point_;
	bool						is_draw_edge_;
	bool						is_move_module_;
	bool						is_draw_face_;
	bool						is_draw_texture_;
	bool						has_lighting_;
	bool						is_draw_axes_;
	bool						is_draw_grid_;
	bool						is_draw_cutpieces_;
	bool						is_select_face;
	bool						is_draw_hatch_;
	bool						is_show_all;
private:
	int slice_check_id_;
	int current_face_;
	Supportbyslice supportor;
	MSAABB box;
	
	void ClearSlice();
	void ClearHatch();
	void ClearSupport();
};

#endif // RENDERINGWIDGET_H
