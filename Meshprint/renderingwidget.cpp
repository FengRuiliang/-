#pragma once
#include "renderingwidget.h"
#include <QKeyEvent>
#include <QColorDialog>
#include <QFileDialog>
#include <iostream>
#include <QtWidgets/QMenu>
#include <QtWidgets/QAction>
#include <QTextCodec>
#include <gl/GLU.h>
#include <gl/glut.h>
#include <algorithm>
#include <queue>
//#include "mainwindow.h"
#include "ArcBall.h"
#include "globalFunctions.h"
//#include "HE_mesh/Mesh3D.h"
#include "Support.h"
#include "SliceCut.h"
#include "openGLProjector.h"
#include "QDebug"
#include "meshprint.h"
#include <fstream>
#include "Hatch.h"
#include "space2dKDTree.h"
#include <QTime>
#include "Grid.h"
#include <omp.h>
#include "MeshInfo.h"
#include <QMessageBox>
#include "maintenance.h"
#include "Supportbyslice.h"
#include "Preprocessor.h"

class Support;

using namespace trimesh;
static GLfloat win, hei;
RenderingWidget::RenderingWidget(QWidget *parent, MainWindow* mainwindow)
	: QOpenGLWidget(parent), ptr_mainwindow_(mainwindow), eye_distance_(50.0),
	has_lighting_(true), is_draw_point_(false), is_draw_edge_(false), is_draw_face_(true)
{
	ptr_arcball_ = new CArcBall(width(), height());
	current_face_ = -1;
	isAddPoint = false;
	isAddLine = false;
	is_select_face = false;
	is_draw_hatch_ = false;
	isDelete = false;
	is_load_texture_ = false;
	is_draw_axes_ = false;
	is_draw_texture_ = (false);
	is_draw_grid_ = (false);
	is_draw_cutpieces_ = (false);
	is_move_module_ = (false);
	is_draw_region_ = false;
	is_show_all = false;
	eye_goal_[0] = eye_goal_[1] = eye_goal_[2] = 0.0;
	eye_direction_[0] = eye_direction_[1] = 0.0;
	eye_direction_[2] = 1.0;
	slice_check_id_ = 1;
	is_draw_support_ = true;
	sphere_for_display.LoadFromOBJFile("./Resources/models/sp_sim.obj");
	sphere_for_display.scalemesh(0.1);
}

RenderingWidget::~RenderingWidget()
{
	SafeDelete(ptr_arcball_);
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

		SafeDelete(ptr_mesh_);
		SafeDelete(ptr_support_);
	}
}

void RenderingWidget::initializeGL()
{
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glShadeModel(GL_FLAT);
	glEnable(GL_DOUBLEBUFFER);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_DIFFUSE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_DEPTH_TEST);
	glClearDepth(1);

	SetLight();
}

void RenderingWidget::resizeGL(int w, int h)
{
	h = (h == 0) ? 1 : h;
	win = w;
	hei = h;
	ptr_arcball_->reSetBound(w, h);
	glViewport(0, 0, (GLfloat)eye_distance_*w, (GLfloat)eye_distance_*h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (w <= h)
		glOrtho(-eye_distance_, eye_distance_, -eye_distance_ * (GLfloat)h / (GLfloat)w, eye_distance_ * (GLfloat)h / (GLfloat)w, -200.0, 200.0);
	else
		glOrtho(-eye_distance_*(GLfloat)w / (GLfloat)h, eye_distance_*(GLfloat)w / (GLfloat)h, -eye_distance_, eye_distance_, -200.0, 200.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void RenderingWidget::paintGL()
{
	GLdouble eqn[4] = { 0.0, 0.0, -1.0, 30};
	glClipPlane(GL_CLIP_PLANE0, eqn);
	
	//glEnable(GL_CLIP_PLANE0);
	glShadeModel(GL_SMOOTH);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (win <= hei)
		glOrtho(-eye_distance_ + eye_goal_[0], eye_distance_ + eye_goal_[0],
			-eye_distance_ * (GLfloat)hei / (GLfloat)win + eye_goal_[1], eye_distance_ * (GLfloat)hei / (GLfloat)win + eye_goal_[1],
			-200.0, 200.0);
	else
		glOrtho(-eye_distance_ * (GLfloat)win / (GLfloat)hei + eye_goal_[0], eye_distance_ * (GLfloat)win / (GLfloat)hei + eye_goal_[0],
			-eye_distance_ + eye_goal_[1], eye_distance_ + eye_goal_[1], -200.0, 200.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (has_lighting_)
	{
		SetLight();
	}
	else
	{
		glDisable(GL_LIGHTING);
		glDisable(GL_LIGHT0);
	}
	glMultMatrixf(ptr_arcball_->GetBallMatrix());
	Render();

}

void RenderingWidget::timerEvent(QTimerEvent * e)
{
	update();
}

void RenderingWidget::mousePressEvent(QMouseEvent *e)
{

	switch (e->button())
	{
	case Qt::LeftButton:
	{
		makeCurrent();
		ptr_arcball_->MouseDown(e->pos());
		break;



		OpenGLProjector myProjector = OpenGLProjector();

		Vec3f myPointN(e->x(), height() - e->y(), -1.0f);
		Vec3f myPointF(e->x(), height() - e->y(), 1.0f);
		Vec3f add_pointN = myProjector.UnProject(myPointN);
		Vec3f add_pointF = myProjector.UnProject(myPointF);


		Vec3f direc = add_pointF - add_pointN;
		add_pointN -= direc * 10.0f;
		direc.normalize();

		Vec3f hitPointT;
		int hitID = -1;
		float temp_t = -1.0f;
		this->ctn_obj[0]->ptr_support_->GetMeshInOctree()->hitOctreeNode(this->ctn_obj[0]->ptr_support_->GetMeshInOctree()->oc_root_, add_pointN, hitPointT, direc, hitID, temp_t);

		if (isAddPoint && hitID != -1)
		{

			const Vec3f _z(0, 0, -1);
			std::vector<HE_vert *> tmp_face_verts;
			this->ctn_obj[0]->ptr_mesh_->get_faces_list()->at(hitID)->face_verts(tmp_face_verts);

			Vec3f eAB = tmp_face_verts[1]->position_ - tmp_face_verts[0]->position_;
			Vec3f eAC = tmp_face_verts[2]->position_ - tmp_face_verts[0]->position_;
			Vec3f vNormal = eAB.cross(eAC);
			vNormal.normalize();

			if (vNormal.dot(_z) <= 0)
			{
				std::cout << "this point is not facing downward" << std::endl;
				break;
			}

			ctn_obj[0]->ptr_support_->AddPointSupport(hitPointT);
			ctn_obj[0]->ptr_support_->updateSupportMesh();
			ctn_obj[0]->ptr_support_->BuildSupportOctree();
			parent->ui.applyPointSupportButton->setChecked(true);
			update();
		}
		else if (isAddLine)
		{
			if (line_points_.size() < 2 && hitID != -1)
			{
				line_points_.push_back(hitPointT);
			}
			if (line_points_.size() == 2)
			{
				//sample points on (line_points_[0], line_points_[1]) and add support structure to the mesh
				float lengthTmp = (line_points_[1] - line_points_[0]).length();
				Vec3f direc = line_points_[1] - line_points_[0];
				direc.normalize();

				std::vector<Vec3f> toAdd;
				float t = 0;
				while (t < lengthTmp)
				{
					Vec3f point1T = line_points_[0] + t * direc;
					////////////////////////// hit UP and DOWN, choose interacted point outside the model //////////////////// 		
					int hitID_up = -1;
					int hitID_down = -1;
					Vec3f point1TUp = ctn_obj[0]->ptr_support_->GetMeshInOctree()->InteractPoint(point1T, Vec3f(0, 0, 1), true, std::vector<int>(), 0, hitID_up);
					Vec3f point1TDo = ctn_obj[0]->ptr_support_->GetMeshInOctree()->InteractPoint(point1T, Vec3f(0, 0, -1), true, std::vector<int>(), 0, hitID_down);
					if (hitID_up != -1)
					{
						if (ctn_obj[0]->ptr_mesh_->get_face(hitID_up)->normal() * Vec3f(0, 0, -1) > 0)
						{
							point1T = point1TUp;
						}
						else if (ctn_obj[0]->ptr_mesh_->get_face(hitID_down)->normal() * Vec3f(0, 0, -1) > 0)
						{
							point1T = point1TDo;
						}
					}
					else
					{
						// hit downside		
						point1T = ctn_obj[0]->ptr_support_->GetMeshInOctree()->InteractPoint(point1T, Vec3f(0, 0, -1), true);
					}
					////////////////////////// hit UP and DOWN, choose interacted point outside the model ////////////////////

					toAdd.push_back(point1T);

					t += RESO;
				}
				toAdd.push_back(line_points_[1]);
				ctn_obj[0]->ptr_support_->AddLineSupport(toAdd);
				ctn_obj[0]->ptr_support_->updateSupportMesh();
				ctn_obj[0]->ptr_support_->BuildSupportOctree();
				line_points_.clear();
				update();
			}
		}
		else
		{
			ptr_arcball_->MouseDown(e->pos());
		}
		update();
	}
	break;
	case Qt::MidButton:
		current_position_ = e->pos();
		break;
	case  Qt::RightButton:
	{
		makeCurrent();
		OpenGLProjector myProjector = OpenGLProjector();
		Vec3f myPointN(e->x(), height() - e->y(), -1.0f);
		Vec3f myPointF(e->x(), height() - e->y(), 1.0f);
		Vec3f add_pointN = myProjector.UnProject(myPointN);
		Vec3f add_pointF = myProjector.UnProject(myPointF);
		Vec3f direc = add_pointF - add_pointN;
		add_pointN -= direc * 10.0f;
		direc.normalize();
		Vec3f hitPointT;
		int hitID = -1;
		float temp_t = -1.0f;
		this->ctn_obj[0]->ptr_support_->GetMeshInOctree()->hitOctreeNode(this->ctn_obj[0]->ptr_support_->GetMeshInOctree()->oc_root_, add_pointN, hitPointT, direc, hitID, temp_t);
		SetDirection(hitID);
		ctn_obj[0]->ptr_mesh_->UpdateMesh();
		ctn_obj[0]->ptr_mesh_->scalemesh(1.0);
		//ctn_obj[0]->ptr_support_->GetMeshInOctree()->DeleteMeshOcTree();
		ctn_obj[0]->ptr_support_->GetMeshInOctree()->BuildOctree(ctn_obj[0]->ptr_mesh_);
		current_position_ = e->pos();
		update();
		break;
	}

	default:
		break;
	}
}
void RenderingWidget::mouseMoveEvent(QMouseEvent *e)
{
	switch (e->buttons())
	{
		setCursor(Qt::ClosedHandCursor);

	case Qt::LeftButton:
		if (isDelete && delete_points_.size() < 1000)
		{
			delete_points_.push_back(e->x());
			delete_points_.push_back(height() - e->y());
		}
		else if (is_move_module_)
		{
		}
		else
		{
			ptr_arcball_->MouseMove(e->pos());
		}
		break;

	case Qt::MidButton:
		if (is_move_module_)
		{
			current_position_ = e->pos();
		}
		else
		{
			if (ctn_obj[0]->ptr_mesh_ != NULL)
			{
				eye_goal_[0] -= 10*GLfloat(e->x() - current_position_.x()) / GLfloat(width());
				eye_goal_[1] += 10*GLfloat(e->y() - current_position_.y()) / GLfloat(height());
			}
			current_position_ = e->pos();
		}
		break;
	default:
		break;
	}
	update();
}
void RenderingWidget::mouseReleaseEvent(QMouseEvent *e)
{
	switch (e->button())
	{
	case Qt::LeftButton:
		if (is_move_module_)
		{
		}
		else
		{
			ptr_arcball_->MouseUp(e->pos());
		}

		setCursor(Qt::ArrowCursor);
		if (isDelete)
		{
			if (!delete_points_.empty())
			{
				makeCurrent();
				ctn_obj[0]->ptr_support_->DeleteSupport(delete_points_);
			}
			delete_points_.clear();
			update();
		}
		else
		{
			ptr_arcball_->MouseUp(e->pos());
			setCursor(Qt::ArrowCursor);
		}
		break;
	case Qt::RightButton:
		break;
	default:
		break;
	}
}
void RenderingWidget::mouseDoubleClickEvent(QMouseEvent *e)
{
	switch (e->button())
	{
	default:
		break;
	}
	update();
}
void RenderingWidget::wheelEvent(QWheelEvent *e)
{
	if (ctn_obj.size())
	{
		eye_distance_ -= e->delta()*ctn_obj[0]->ptr_mesh_->getBoundingBox().at(0).at(2) / 1000;
	}
	eye_distance_ = eye_distance_ < 0 ? 0 : eye_distance_;
	update();
}

void RenderingWidget::keyPressEvent(QKeyEvent *e)
{
	switch (e->key())
	{
	case Qt::Key_A:
		break;
	default:
		break;
	}
}

void RenderingWidget::keyReleaseEvent(QKeyEvent *e)
{
	switch (e->key())
	{
	case Qt::Key_A:
		break;
	default:
		break;
	}
}

void RenderingWidget::Render()
{
	DrawAxes(is_draw_axes_);
	DrawGrid(is_draw_grid_);
	DrawPoints(is_draw_point_);
	//DrawEdge(is_draw_edge_);
	DrawFace(is_draw_face_);
	DrawCutPieces(is_draw_edge_);
	DrawSupport(is_draw_support_);
	//DrawHatchsup(true);
}

void RenderingWidget::SetLight()
{
	//return;
	static GLfloat mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	static GLfloat mat_shininess[] = { 50.0f };
	static GLfloat light_position0[] = { 0.0, 0.0, 100.0, 0.0f };
	static GLfloat light_position1[] = { -1.0f, -1.0f, 0.5f, 0.0f };
	static GLfloat light_position2[] = { 0.0, 0.0, -100.0, 0.0f };
	static GLfloat bright[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	static GLfloat dim_light[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	static GLfloat lmodel_ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };

	//glMaterialfv(GL_FRONT, GL_AMBIENT, mat_specular);
	//glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_specular);
	//glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	//glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, bright);
	glLightfv(GL_LIGHT0, GL_SPECULAR, bright);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, dim_light);
	//glLightfv(GL_LIGHT1, GL_SPECULAR, white_light);
	//glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightfv(GL_LIGHT2, GL_POSITION, light_position2);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, bright);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
}

bool booladsad = true;
void RenderingWidget::SetBackground()
{
	QColor color = QColorDialog::getColor(Qt::white, this, tr("background color"));
	GLfloat r = (color.red()) / 255.0f;
	GLfloat g = (color.green()) / 255.0f;
	GLfloat b = (color.blue()) / 255.0f;
	GLfloat alpha = color.alpha() / 255.0f;
	makeCurrent();
	glClearColor(r, g, b, alpha);

	//updateGL();
	update();
}

void RenderingWidget::ReadSingleMesh()
{
	QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
	QString str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
	ptr_arcball_->reSetBound(width(), height());

	ctn_obj.clear();
	ctn_obj.push_back(new AMObject);
	ctn_obj.back() = new AMObject();
	ctn_obj.back()->ptr_mesh_ = new Mesh3D;
	is_draw_grid_ = true;
	is_draw_face_ = true;
	is_draw_cutpieces_ = true;
	is_draw_hatch_ = true;
	has_lighting_ = true;
	//ClearSlice();
	//ClearHatch();
	QString filename = QFileDialog::
		getOpenFileName(this, tr("Read Mesh"),
			"\models", tr("Meshes (*.obj *.stl)"));

	if (filename.isEmpty())
	{
		QMessageBox::information(this, "Warning Message!", "Please select a file");
		return;
	}
	//中文路径支持
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);

	//mycut->clearcut();

	QByteArray byfilename = filename.toLocal8Bit();
	QFileInfo fileinfo = QFileInfo(filename);
	//qDebug() << "read Mesh start time" << str;
	//qDebug() << byfilename.data();
	qDebug() << "load model time at " << time;

	if (fileinfo.suffix() == "obj")
	{
		ctn_obj.back()->ptr_mesh_->LoadFromOBJFile(byfilename.data());
	}
	else if (fileinfo.suffix() == "stl" || fileinfo.suffix() == "STL")
	{
		ctn_obj.back()->ptr_mesh_->LoadFromSTLFile(byfilename.data());
	}
	ctn_obj.back()->ptr_mesh_->markEdges();
	float max_ = ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).at(0);
	max_ = max_ > ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).at(1) ? max_ : ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).at(1);
	max_ = max_ > ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).at(2) ? max_ : ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).at(2);


	ctn_obj.back()->ptr_support_ = new Support(ctn_obj.back()->ptr_mesh_);
	ctn_obj.back()->ptr_support_->GetMeshInOctree()->BuildOctree(ctn_obj.back()->ptr_mesh_);

	QString a = "this model has ";
	a.append(QString::number(ctn_obj.back()->ptr_mesh_->num_of_face_list(), 10));
	a.append(" faces, and it's size is ");
	double v1 = ctn_obj.back()->ptr_mesh_->getBoundingBox()[0][0] * 2;
	double v2 = ctn_obj.back()->ptr_mesh_->getBoundingBox()[0][1] * 2;
	double v3 = ctn_obj.back()->ptr_mesh_->getBoundingBox()[0][2];
	QString s = QString("%1").arg(v1, 0, 'g', 4);
	a.append(s);
	a.append(" x ");
	s = QString("%1").arg(v2, 0, 'g', 4);
	a.append(s);
	a.append(" x ");
	s = QString("%1").arg(v3, 0, 'g', 4);
	a.append(s);

	QMessageBox::information(this, "Read Successfully!", a);
	float x_length = 0; float y_length = 0;
	for (int i = 0; i < ctn_obj.size(); i++)
	{
		float c_x = ctn_obj[i]->ptr_mesh_->getBoundingBox()[0].x() - ctn_obj[i]->ptr_mesh_->getBoundingBox()[1].x();
		float c_y = ctn_obj[i]->ptr_mesh_->getBoundingBox()[0].y() - ctn_obj[i]->ptr_mesh_->getBoundingBox()[1].y();
		x_length = x_length > c_x ? x_length : c_x;
		y_length = y_length > c_y ? y_length : c_y;
	}

	MSAABB bb;

	int matri = 0;
	while (matri*matri < ctn_obj.size())
		matri++;

	for (int i = 0; i < ctn_obj.size(); i++)
	{
		ctn_obj[i]->ptr_mesh_->UpdateMesh();
		int row = i / matri;
		int col = i % matri;
		ctn_obj[i]->ptr_mesh_->meshTranslate(row*x_length, col*y_length);
		bb.merge(MSAABB(ctn_obj[i]->ptr_mesh_->getBoundingBox()[0], ctn_obj[i]->ptr_mesh_->getBoundingBox()[1]));
	}
	for (int i = 0; i < ctn_obj.size(); i++)
	{
		ctn_obj[i]->ptr_mesh_->meshTranslate(-bb.getCenter().x(), -bb.getCenter().y());
	}
}

void RenderingWidget::ReadMesh()
{
	QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
	QString str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
	ptr_arcball_->reSetBound(width(), height());
	ctn_obj.push_back(new AMObject);
	//ctn_obj.back() = new AMObject();
	ctn_obj.back()->ptr_mesh_ = new Mesh3D;
	

	is_draw_grid_ = true;
	is_draw_face_ = true;
	is_draw_cutpieces_ = true;
	is_draw_hatch_ = true;
	has_lighting_ = true;
	//ClearSlice();
	//ClearHatch();
	QString filename = QFileDialog::
		getOpenFileName(this, tr("Read Mesh"),
			"\models", tr("Meshes (*.obj *.stl)"));

	if (filename.isEmpty())
	{
		QMessageBox::information(this, "Warning Message!", "Please select a file");
		return;
	}
	//中文路径支持
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);

	//mycut->clearcut();

	QByteArray byfilename = filename.toLocal8Bit();
	QFileInfo fileinfo = QFileInfo(filename);
	//qDebug() << "read Mesh start time" << str;
	//qDebug() << byfilename.data();
	qDebug() << "load model time at " << time;

	if (fileinfo.suffix() == "obj")
	{
		ctn_obj.back()->ptr_mesh_->LoadFromOBJFile(byfilename.data());
	}
	else if (fileinfo.suffix() == "stl" || fileinfo.suffix() == "STL")
	{
		ctn_obj.back()->ptr_mesh_->LoadFromSTLFile(byfilename.data());
	}
	ctn_obj.back()->ptr_mesh_->markEdges();
	float max_ = ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).at(0);
	max_ = max_ > ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).at(1) ? max_ : ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).at(1);
	max_ = max_ > ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).at(2) ? max_ : ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).at(2);


	ctn_obj.back()->ptr_support_ = new Support(ctn_obj.back()->ptr_mesh_);
	ctn_obj.back()->ptr_support_->GetMeshInOctree()->BuildOctree(ctn_obj.back()->ptr_mesh_);

	QString a = "this model has ";
	a.append(QString::number(ctn_obj.back()->ptr_mesh_->num_of_face_list(), 10));
	a.append(" faces, and it's size is ");
	double v1 = ctn_obj.back()->ptr_mesh_->getBoundingBox()[0][0] * 2;
	double v2 = ctn_obj.back()->ptr_mesh_->getBoundingBox()[0][1] * 2;
	double v3 = ctn_obj.back()->ptr_mesh_->getBoundingBox()[0][2];
	QString s = QString("%1").arg(v1, 0, 'g', 4);
	a.append(s);
	a.append(" x ");
	s = QString("%1").arg(v2, 0, 'g', 4);
	a.append(s);
	a.append(" x ");
	s = QString("%1").arg(v3, 0, 'g', 4);
	a.append(s);

	//QMessageBox::information(this, "Read Successfully!", a);
	float x_length = 0; float y_length = 0;
	for (int i = 0; i < ctn_obj.size(); i++)
	{
		float c_x = ctn_obj[i]->ptr_mesh_->getBoundingBox()[0].x() - ctn_obj[i]->ptr_mesh_->getBoundingBox()[1].x();
		float c_y = ctn_obj[i]->ptr_mesh_->getBoundingBox()[0].y() - ctn_obj[i]->ptr_mesh_->getBoundingBox()[1].y();
		x_length = x_length > c_x ? x_length : c_x;
		y_length = y_length > c_y ? y_length : c_y;
	}

	MSAABB bb;

	int matri = 0;
	while (matri*matri < ctn_obj.size())
		matri++;

	for (int i = 0; i < ctn_obj.size(); i++)
	{
		ctn_obj[i]->ptr_mesh_->UpdateMesh(); 
		int row = i / matri;
		int col = i % matri;
		ctn_obj[i]->ptr_mesh_->meshTranslate(row*x_length, col*y_length);
		bb.merge(MSAABB (ctn_obj[i]->ptr_mesh_->getBoundingBox()[0], ctn_obj[i]->ptr_mesh_->getBoundingBox()[1]));
	}
	for (int i = 0; i < ctn_obj.size(); i++)
	{
		ctn_obj[i]->ptr_mesh_->meshTranslate(-bb.getCenter().x(), -bb.getCenter().y());
	}
}

void RenderingWidget::WriteMesh()
{
	for (int i = 0; i < ctn_obj.size(); i++)
	{
		QString filename = QFileDialog::
			getSaveFileName(this, tr("Write Mesh"),
				"..", tr("Meshes (*.obj)"));

		if (filename.isEmpty())
			return;

		QByteArray byfilename = filename.toLocal8Bit();
		ctn_obj[i]->ptr_mesh_->WriteToOBJFile(byfilename);

	}
}

void RenderingWidget::SaveContour()
{
	for (int i = 0; i < ctn_obj.size(); i++)
	{
		QString filename = QFileDialog::
			getSaveFileName(this, tr("Write contour"),
				"..", tr("Meshes (*.txt)"));

		if (filename.isEmpty())
			return;

		QByteArray byfilename = filename.toLocal8Bit();
		ctn_obj[i]->mycut->write_contour_to_txt(byfilename,slice_check_id_);

	}
}

void RenderingWidget::Export()
{
	if (ctn_obj.empty())
	{
		return;
	}
	QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
	QString str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
															//qDebug() << "export AFF file start time" << str;
	QString filename = QFileDialog::
		getSaveFileName(this, tr("export hatch"),
			"..", tr("aff (*.aff)"));
	if (filename.isEmpty())
		return;

	QByteArray byfilename = filename.toLocal8Bit();
	QFile file(byfilename);
	file.open(QIODevice::WriteOnly);
	file.resize(0);
	char *ptr;


	int layers = ctn_obj.back()->mycut->GetNumPieces() - 1;
	float power = ctn_obj.back()->myhatch->getLaserPower();
	float xmin = ctn_obj.back()->ptr_mesh_->getBoundingBox().at(1).x();
	float xmax = ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).x();
	float ymin = ctn_obj.back()->ptr_mesh_->getBoundingBox().at(1).y();
	float ymax = ctn_obj.back()->ptr_mesh_->getBoundingBox().at(0).y();
	int zmin = 300;
	int zmax = (ctn_obj.back()->myhatch->GetNumPieces() - 1) * 300;
	float speed = ctn_obj.back()->myhatch->getLaserSpeed();
	thickness_ = ctn_obj.back()->mycut->getThickness();
	int  THICKNESS = thickness_ * 10000;
	QDataStream outBinary(&file);
	outBinary.setVersion(QDataStream::Qt_4_1);
	QByteArray* ts = new QByteArray[17];
	ts[0] = "<Header type=\"Autofab Buildfile\" version=\"1.0\">\n";
	ts[1] = "<Origin>HT 149</Origin>\n";
	ts[2] = "<Layers>";
	ts[2].append(QString("%1").arg(layers));
	ts[2].append("</Layers>\n");
	ts[3] = "<Bounds xmin=\"";
	ts[3].append(QString("%1").arg(xmin));
	ts[3].append("\" ymin=\"");
	ts[3].append(QString("%1").arg(ymin));
	ts[3].append("\" xmax=\"");
	ts[3].append(QString("%1").arg(xmax));
	ts[3].append("\" ymax=\"");
	ts[3].append(QString("%1").arg(ymax));
	ts[3].append("\"></Bounds>\n");
	ts[4] = "<Zunit>10000</Zunit>\n";
	ts[5] = "<Zmin>";
	ts[5].append(QString("%1").arg(zmin));
	ts[5].append("</Zmin>\n");
	ts[6] = "<Zmax>";
	ts[6].append(QString("%1").arg(zmax));
	ts[6].append("</Zmax>\n");
	ts[7] = "<LayerThickness>";
	ts[7].append(QString("%1").arg(THICKNESS));
	ts[7].append("</LayerThickness>\n");
	ts[8] = "<Machine>My Own Fabber</Machine>\n";
	ts[9] = "<Material>StainlessSteel_100</Material>\n";
	ts[10] = "<Part name=\"PAWN\" id=\"1\"></Part>\n";
	ts[11] = "<Part name=\"KING\" id=\"2\"></Part>\n";
	ts[12] = "<VectorAttribute name=\"LaserPower\" id=\"6\"></VectorAttribute>\n";
	ts[13] = "<VectorAttribute name=\"Speed\" id=\"7\"></VectorAttribute>\n";
	ts[14] = "<VectorAttribute name=\"Focus\" id=\"8\"></VectorAttribute>\n";
	ts[15] = "<VectorAttribute name=\"PartId\" id=\"11\"></VectorAttribute>\n";
	ts[16] = "</Header>\r\n";
	for (int i = 0; i < 17; i++)
	{
		outBinary.writeRawData(ts[i], sizeof(char) * (ts[i].size()));
	}

	int Layer_Section = 1;
	int LayerZpos_Section = 12;
	int Polygon_Section = 2;
	int Hatch_Section = 4;
	int LaserPower_Section = 6;
	int LaserSpeed_Section = 7;
	int FocusShift_Section = 8;
	int PolygonCoordinates_Section = 3;
	int HatchCoordinates_Section = 5;
	int PartID_Section = 5;
	float laserpower = 0;
	float laserspeed = 0;

	for (int i = 1; i < 100; i++)
	{
		for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
		{
			SliceCut* mycut = ctn_obj[id_obj]->mycut;
			SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
			Hatch* myhatch = ctn_obj[id_obj]->myhatch;
			Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
			Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
			Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

			std::vector<Vec3f *>* tc = myhatch->getHatch();
			std::vector < std::vector<cutLine>* >*tc2 = mycut->GetPieces();
			std::vector < std::vector<Vec3f>>* tc3 = myhatch->getOffsetVertex();
			std::vector<Vec3f *>* tc_s_hatch_ = NULL;
			std::vector < std::vector<cutLine>* >* tc_s_pieces_ = NULL;
			std::vector < std::vector<Vec3f>>* tc_s_offsetvertex_ = NULL;
			if (mycutsup != NULL)
			{
				tc_s_pieces_ = (mycutsup->GetPieces());
			}
			if (myhatchsup != NULL)
			{
				tc_s_hatch_ = myhatchsup->getHatch();
				tc_s_offsetvertex_ = myhatchsup->getOffsetVertex();
			}

			int zposition = i*thickness_ * 10000;
			int len_layer = 0;
			int len_Zpos = 4;
			int len_Polygon = 0;
			int len_Hatch = 0;
			int len_LaserPower = 4;
			int len_laserSpeed = 4;
			int len_Part = 2;
			int*len_PolygonCoor = NULL;
			int len_HatchCoor = 0;
			QByteArray newlayer = NULL;
			QByteArray zPosition = NULL;
			QByteArray hatch = NULL;
			QByteArray hatchCoor = NULL;
			QByteArray polygon = NULL;
			QByteArray* polygonCoor = NULL;
			QByteArray laserPower = NULL;
			QByteArray laserSpeed = NULL;
			QByteArray laserPowerPolygon = NULL;
			QByteArray laserSpeedPolygon = NULL;
			QByteArray partID = NULL;
			//write all hatch coordinate section
			len_HatchCoor = tc[i].size() * 2 * 8;//the length of hatch coordinate section 
			hatchCoor.append(reinterpret_cast<const char *>(&HatchCoordinates_Section), 2);

			hatchCoor.append(reinterpret_cast<const char *>(&len_HatchCoor), 4);
			for (auto iterhatch_ = tc[i].begin(); iterhatch_ != tc[i].end(); iterhatch_++)
			{
				hatchCoor.append(reinterpret_cast<const char *>(&(*iterhatch_)[0].x()), sizeof(float));
				hatchCoor.append(reinterpret_cast<const char *>(&(*iterhatch_)[0].y()), sizeof(float));
				hatchCoor.append(reinterpret_cast<const char*>(&(*iterhatch_)[1].x()), sizeof(float));
				hatchCoor.append(reinterpret_cast<const char*>(&(*iterhatch_)[1].y()), sizeof(float));
			}
			//write all hatch section
			laserPower.append(reinterpret_cast<const char *>(&LaserPower_Section), 2);
			laserPower.append(reinterpret_cast<const char *>(&len_LaserPower), 4);
			laserPower.append(reinterpret_cast<const char *>(&laser_power_hatch_), 4);
			laserSpeed.append(reinterpret_cast<const char *>(&LaserSpeed_Section), 2);
			laserSpeed.append(reinterpret_cast<const char *>(&len_laserSpeed), 4);
			laserSpeed.append(reinterpret_cast<const char *>(&laser_speed_hatch_), 4);
			//partID.append(reinterpret_cast<const char *>(&PartID_Section), 2);
			len_Hatch = len_HatchCoor + 6 + len_LaserPower + 6 + len_laserSpeed + 6;//the length of hatch section,include laser power length,laser speed length
			hatch.append(reinterpret_cast<const char *>(&Hatch_Section), 2);
			hatch.append(reinterpret_cast<const char *>(&len_Hatch), 4);
			hatch.append(laserPower);
			hatch.append(laserSpeed);
			//hatch.append(partID);
			hatch.append(hatchCoor);

			// write all polygon Coordinates
			len_PolygonCoor = new int[tc3[i].size()];
			int tempLenPolyCoor = 0;
			int tempAllLenPoly = 0;
			for (int j1 = 0; j1 < tc3[i].size(); j1++)
			{
				len_PolygonCoor[j1] = (tc3[i])[j1].size() * 8;
				tempLenPolyCoor += len_PolygonCoor[j1] + 6;
			}
			polygonCoor = new QByteArray[tc3[i].size()];
			for (int j = 0; j < tc3[i].size(); j++)
			{
				polygonCoor[j].append(reinterpret_cast<const char *>(&PolygonCoordinates_Section), 2);//here only one polygon coordinate section
				polygonCoor[j].append(reinterpret_cast<const char *>(&len_PolygonCoor[j]), 4);
				for (int m = 0; m < (tc3[i])[j].size(); m++)
				{
					polygonCoor[j].append(reinterpret_cast<const char *>(&((tc3[i])[j]).at(m).x()), 4);
					polygonCoor[j].append(reinterpret_cast<const char *>(&((tc3[i])[j]).at(m).y()), 4);
				}
			}
			//write laser power and speed to polygon
			laserPowerPolygon.append(reinterpret_cast<const char *>(&LaserPower_Section), 2);
			laserPowerPolygon.append(reinterpret_cast<const char *>(&len_LaserPower), 4);
			laserPowerPolygon.append(reinterpret_cast<const char *>(&laser_power_polygon_), 4);
			laserSpeedPolygon.append(reinterpret_cast<const char *>(&LaserSpeed_Section), 2);
			laserSpeedPolygon.append(reinterpret_cast<const char *>(&len_laserSpeed), 4);
			laserSpeedPolygon.append(reinterpret_cast<const char *>(&laser_speed_polygon_), 4);
			len_Polygon = len_LaserPower + 6 + len_laserSpeed + 6 + tempLenPolyCoor;
			polygon.append(reinterpret_cast<const char *>(&Polygon_Section), 2);
			polygon.append(reinterpret_cast<const char *>(&len_Polygon), 4);
			polygon.append(laserPowerPolygon);
			polygon.append(laserSpeedPolygon);
			for (size_t j2 = 0; j2 < tc3[i].size(); j2++)
			{
				polygon.append(polygonCoor[j2]);
			}
			//here only one polygon coordinate section
			//write z position
			zPosition.append(reinterpret_cast<const char *>(&LayerZpos_Section), 2);
			zPosition.append(reinterpret_cast<const char *>(&len_Zpos), 4);
			zPosition.append(reinterpret_cast<const char *>(&zposition), 4);

			//////////////////////////////////////////////////////////////////////////
			// write support hatch 
			QByteArray hatch_coor_s;
			QByteArray hatch_s_;
			QByteArray* polygon_coor_s;
			QByteArray polygon_s_;

			int len_HatchCoor_s_ = 0;
			int len_Hatch_s_ = 0;
			int* len_PolygonCoor_s_ = NULL;
			int tempLenPolyCoor_s_ = 0;
			int tempAllLenPoly_s_ = 0;
			int	len_Polygon_s_ = 0;
			if (myhatchsup != NULL&&i < myhatchsup->GetNumPieces())
			{
				len_HatchCoor_s_ = tc_s_hatch_[i].size() * 2 * 8;//the length of hatch coordinate section 
				hatch_coor_s.append(reinterpret_cast<const char *>(&HatchCoordinates_Section), 2);
				hatch_coor_s.append(reinterpret_cast<const char *>(&len_HatchCoor_s_), 4);
				for (auto iter_hatch_s_ = tc_s_hatch_[i].begin(); iter_hatch_s_ != tc_s_hatch_[i].end(); iter_hatch_s_++)
				{
					hatch_coor_s.append(reinterpret_cast<const char *>(&(*iter_hatch_s_)[0].x()), sizeof(float));
					hatch_coor_s.append(reinterpret_cast<const char *>(&(*iter_hatch_s_)[0].y()), sizeof(float));
					hatch_coor_s.append(reinterpret_cast<const char*>(&(*iter_hatch_s_)[1].x()), sizeof(float));
					hatch_coor_s.append(reinterpret_cast<const char*>(&(*iter_hatch_s_)[1].y()), sizeof(float));
				}
				len_Hatch_s_ = len_HatchCoor_s_ + 6 + len_LaserPower + 6 + len_laserSpeed + 6;//the length of hatch section,include laser power length,laser speed length

				hatch_s_.append(reinterpret_cast<const char *>(&Hatch_Section), 2);
				hatch_s_.append(reinterpret_cast<const char *>(&len_Hatch_s_), 4);
				hatch_s_.append(laserPower);
				hatch_s_.append(laserSpeed);
				hatch_s_.append(hatch_coor_s);
				//write support polygon
				len_PolygonCoor_s_ = new int[tc_s_offsetvertex_[i].size()];
				tempLenPolyCoor_s_ = 0;
				tempAllLenPoly_s_ = 0;
				for (int j1 = 0; j1 < tc_s_offsetvertex_[i].size(); j1++)
				{
					len_PolygonCoor_s_[j1] = (tc_s_offsetvertex_[i])[j1].size() * 8;
					tempLenPolyCoor_s_ += len_PolygonCoor_s_[j1] + 6;

				}
				polygon_coor_s = new QByteArray[tc_s_offsetvertex_[i].size()];
				for (int j = 0; j < tc_s_offsetvertex_[i].size(); j++)
				{
					polygon_coor_s[j].append(reinterpret_cast<const char *>(&PolygonCoordinates_Section), 2);//here only one polygon coordinate section
					polygon_coor_s[j].append(reinterpret_cast<const char *>(&len_PolygonCoor_s_[j]), 4);
					for (int m = 0; m < (tc_s_offsetvertex_[i])[j].size(); m++)
					{
						polygon_coor_s[j].append(reinterpret_cast<const char *>(&((tc_s_offsetvertex_[i])[j]).at(m).x()), 4);
						polygon_coor_s[j].append(reinterpret_cast<const char *>(&((tc_s_offsetvertex_[i])[j]).at(m).y()), 4);
					}
				}
				//write laser power and speed to polygon
				len_Polygon_s_ = len_LaserPower + 6 + len_laserSpeed + 6 + tempLenPolyCoor_s_;
				polygon_s_.append(reinterpret_cast<const char *>(&Polygon_Section), 2);
				polygon_s_.append(reinterpret_cast<const char *>(&len_Polygon_s_), 4);
				polygon_s_.append(laserPowerPolygon);
				polygon_s_.append(laserSpeedPolygon);
				for (size_t j2 = 0; j2 < tc_s_offsetvertex_[i].size(); j2++)
				{
					polygon_s_.append(polygon_coor_s[j2]);
				}

			}
			//////////////////////////////////////////////////////////////////////////
			if (myhatchsup != NULL)
			{
				len_layer = len_Zpos + 6 + len_Hatch + 6 + len_Polygon + 6 + len_Hatch_s_ + 6 + len_Polygon_s_ + 6;
			}
			else
			{
				len_layer = len_Zpos + 6 + len_Hatch + 6 + len_Polygon + 6;

			}

			newlayer.append(reinterpret_cast<const char *>(&Layer_Section), 2);
			newlayer.append(reinterpret_cast<const char *>(&len_layer), 4);
			newlayer.append(zPosition);
			newlayer.append(hatch);
			newlayer.append(polygon);
			if (myhatchsup != NULL)
			{
				newlayer.append(hatch_s_);
				newlayer.append(polygon_s_);
			}
			outBinary.writeRawData(newlayer, sizeof(char) * (newlayer.size()));
			delete[]polygonCoor;
			delete[]len_PolygonCoor;
		}
	}
	file.flush();
	file.close();
	time = QDateTime::currentDateTime();//获取系统现在的时间
	str = time.toString("yyyy-MM-dd hh:mm:ss ddd"); //设置显示格式
													//qDebug() << "export AFF file end time :" << str;
	SafeDeletes(ts);
}

void RenderingWidget::LoadTexture()
{
	QString filename = QFileDialog::getOpenFileName(this, tr("Load Texture"),
		"..", tr("Images(*.bmp *.jpg *.png *.jpeg)"));
	if (filename.isEmpty())
	{
		//emit(operatorInfo(QString("Load Texture Failed!")));
		return;
	}


	glGenTextures(1, &texture_[0]);
	QImage tex1, buf;
	if (!buf.load(filename))
	{
		//        QMessageBox::warning(this, tr("Load Fialed!"), tr("Cannot Load Image %1").arg(filenames.at(0)));
		//emit(operatorInfo(QString("Load Texture Failed!")));
		return;

		//QImage dummy(128, 128, QImage::Format_ARGB32);
		//dummy.fill(Qt::green);
		//buf = dummy;
		//
	}

	glBindTexture(GL_TEXTURE_2D, texture_[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_NEAREST);
	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, tex1.width(), tex1.height(),
		GL_RGBA, GL_UNSIGNED_BYTE, tex1.bits());

	is_load_texture_ = true;
	//emit(operatorInfo(QString("Load Texture from ") + filename + QString(" Done")));
}


void RenderingWidget::SetSliceCheckId(int id)
{
	slice_check_id_ = id;
	update();
}

void RenderingWidget::CheckDrawPoint()
{
	is_draw_point_ = !is_draw_point_;
	//updateGL();
	update();

}

void RenderingWidget::CheckDrawEdge()
{
	is_draw_edge_ = !is_draw_edge_;
	update();

}

void RenderingWidget::CheckDrawFace()
{
	is_draw_face_ = !is_draw_face_;

	//updateGL();
	update();

}
void RenderingWidget::CheckLight()
{
	has_lighting_ = !has_lighting_;
	is_draw_support_ = has_lighting_;
	//updateGL();
	update();

}

void RenderingWidget::CheckGrid()
{
	is_draw_grid_ = !is_draw_grid_;
	//qDebug() << is_draw_grid_;
	//updateGL();
	update();

}

void RenderingWidget::CheckDrawTexture()
{
	is_draw_texture_ = !is_draw_texture_;
	if (is_draw_texture_)
		glEnable(GL_TEXTURE_2D);
	else
		glDisable(GL_TEXTURE_2D);

	//updateGL();
	update();

}

void RenderingWidget::CheckDrawAxes()
{
	is_draw_axes_ = !is_draw_axes_;
	//updateGL();
	update();

}

void RenderingWidget::CheckDrawCutPieces()
{

	is_draw_cutpieces_ = true;
	//updateGL();
	update();


}

void RenderingWidget::Checkmoduletranslate()
{
	is_move_module_ = !is_move_module_;
	//updateGL();
	update();

}

void RenderingWidget::CheckSetFace()
{
	is_select_face = !is_select_face;
	//updateGL();
	update();


}

void RenderingWidget::CheckRegion(bool bv)
{
	is_draw_region_ = bv;
	//updateGL();
	update();
}

void RenderingWidget::CheckSupport(bool bv)
{
	is_draw_support_ = bv;
	qDebug() << is_draw_support_;
	//updateGL();
	update();
}

void RenderingWidget::CheckRotateModel(bool bv)
{
	is_move_module_ = bv;
}


std::vector<int> qwewqe1(500000);
std::vector<int> qwewqe(500000);

void RenderingWidget::DrawAxes(bool bv)
{
	if (!bv)
		return;
	//x axis
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.7, 0.0, 0.0);
	glEnd();
	glPushMatrix();
	glTranslatef(0.7, 0, 0);
	glRotatef(90, 0.0, 1.0, 0.0);
	//glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	//y axis
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.0, 0.7, 0.0);
	glEnd();

	glPushMatrix();
	glTranslatef(0.0, 0.7, 0);
	glRotatef(90, -1.0, 0.0, 0.0);
	//glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	//z axis
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.0, 0.0, 0.7);
	glEnd();
	glPushMatrix();
	glTranslatef(0.0, 0, 0.7);
	//glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	glColor3f(1.0, 1.0, 1.0);
}
void RenderingWidget::DrawPoints(bool bv)
{
	if (!bv || ctn_obj.empty())
		return;
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		const std::vector<HE_vert*>& verts = *(ctn_obj[id_obj]->ptr_mesh_->get_vertex_list());
		//glColor3f(0, 0, 0);
		//glPointSize(1);
		glBegin(GL_POINTS);
		for (size_t i = 0; i != ctn_obj[id_obj]->ptr_mesh_->num_of_vertex_list(); ++i)
		{
			glNormal3fv(verts[i]->normal().data());
			glVertex3fv((verts[i]->position()*scaleV).data());
		}
		glEnd();
		if (ctn_obj[id_obj]->ptr_support_->GetMeshSupport()->num_of_face_list() == 0)
		{
			return;
		}
		const std::vector<HE_vert*>& verts1 = *(ctn_obj[id_obj]->ptr_support_->GetMeshSupport()->get_vertex_list());
		//glColor3f(0, 0, 0);
		//glPointSize(1);
		glBegin(GL_POINTS);
		for (size_t i = 0; i != ctn_obj[id_obj]->ptr_support_->GetMeshSupport()->num_of_vertex_list(); ++i)
		{
			glNormal3fv(verts1[i]->normal().data());
			glVertex3fv((verts1[i]->position()*scaleV).data());
		}
		glEnd();

	}
}
void RenderingWidget::DrawEdge(bool bv)
{
	if (!bv || ctn_obj.empty())
		return;
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		if (ctn_obj[id_obj]->ptr_mesh_->num_of_face_list() == 0)
		{
			return;
		}

		const std::vector<HE_edge *>& edges = *(ctn_obj[id_obj]->ptr_mesh_->get_edges_list());
		const std::vector<HE_edge *>& bedges = *(ctn_obj[id_obj]->ptr_mesh_->get_bedges_list());
		//glLineWidth(5.0);
		for (size_t i = 0; i != bedges.size(); ++i)
		{
			glBegin(GL_LINES);
			glColor3f(1.0, 0.0, 0.0);
			glNormal3fv(bedges[i]->start_->normal().data());
			glVertex3fv((bedges[i]->start_->position()*scaleV).data());
			glNormal3fv(bedges[i]->pvert_->normal().data());
			glVertex3fv((bedges[i]->pvert_->position()*scaleV).data());
			glEnd();
		}
		//continue;
		glBegin(GL_LINES);
		glColor4ub(255, 255, 255, 255);
		for (size_t i = 0; i != edges.size(); ++i)
		{
			glVertex3f(edges[i]->start_->position().x(), edges[i]->start_->position().y(),0);
			glVertex3f(edges[i]->pvert_->position().x(), edges[i]->pvert_->position().y(), 0);

		}
		glEnd();
		
		auto bl = ctn_obj[id_obj]->ptr_mesh_->GetBLoop();
		for (size_t i = 0; i != bl.size(); i++)
		{
			glBegin(GL_LINE_LOOP);
			glColor3f(1.0, 0.0, 0.0);
			for (int j = 0; j < bl[i].size(); j++)
			{
				//glNormal3fv(bl[i][j]->start_->normal().data());
				glVertex3f(bl[i][j]->start_->position().x(), bl[i][j]->start_->position().y(),0);
			}
			glEnd();
		}
		if (ctn_obj[id_obj]->ptr_support_->GetMeshSupport()->num_of_face_list() == 0)
		{
			return;
		}
		const std::vector<HE_edge *>& edges1 = *(ctn_obj[id_obj]->ptr_support_->GetMeshSupport()->get_edges_list());
		const std::vector<HE_edge *>& bedges1 = *(ctn_obj[id_obj]->ptr_support_->GetMeshSupport()->get_bedges_list());
	}
}
void RenderingWidget::DrawFace(bool bv)
{
	if (!bv || ctn_obj.empty())
		return;
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{

		if (ctn_obj[id_obj]->ptr_mesh_->num_of_face_list() == 0)
		{
			return;
		}

		const std::vector<HE_face *>& faces = *(ctn_obj[id_obj]->ptr_mesh_->get_faces_list());
				
		auto faceNeedSupport=ctn_obj[id_obj]->ptr_support_->faceNeedSupport;
		glBegin(GL_TRIANGLES);
		for (size_t i = 0; i < faceNeedSupport.size(); ++i)
		{
			if (faceNeedSupport[i] && is_draw_region_)
			{
				glColor4ub(170, 0, 0, 255);
			}
			else 
			{
				glColor4ub(164, 160,155, 255);

			}
			
			HE_edge *pedge(faces[i]->pedge_);
			glNormal3fv(faces[i]->normal());
			
			do
			{
				glVertex3f(pedge->pvert_->position().x(), pedge->pvert_->position().y(), pedge->pvert_->position().z());
				pedge = pedge->pnext_;

			} while (pedge != faces[i]->pedge_);
			
		}
		glEnd();
	}
}
void RenderingWidget::DrawSupport(bool bv)
{
	auto faces = sphere_for_display.get_faces_list();
	
	
	for (int i = 0; i < ctn_obj.size(); i++)
	{
		if (ctn_obj[i]->ppcs!=NULL&&ctn_obj[i]->ppcs->su!=NULL)
		{
			glColor4ub(228, 26, 28, 255);
			glBegin(GL_TRIANGLES);
			auto points = ctn_obj[i]->ppcs->su->get_sup_points();
			for (int j=0;j<points->size();j++)
			{
// 				if (j != slice_check_id_)
// 				{
// 					continue;
// 				}
				for (int k = 0; k < points->at(j).size(); k++)
				{
					for (auto iterf = faces->begin(); iterf != faces->end(); iterf++)
					{
						HE_edge* sta = (*iterf)->pedge_;
						HE_edge* cur = sta;
						do
						{
							glVertex3fv(cur->pvert_->position() + points->at(j)[k]);
							cur = cur->pnext_;
						} while (cur != sta);
					}
				}

			}
			glEnd();
			auto sup_lines = ctn_obj[i]->ppcs->su->get_suplines();
			for (int j=0;j<sup_lines->size();j++)
			{
				for (int u = 0; u < sup_lines->at(j).size(); u++)
				{
					if (sup_lines->at(j)[u].size()==1)
					{
						glBegin(GL_TRIANGLES);
						for (auto iterf = faces->begin(); iterf != faces->end(); iterf++)
						{
							HE_edge* sta = (*iterf)->pedge_;
							HE_edge* cur = sta;
							do
							{
								glVertex3fv(cur->pvert_->position() + sup_lines->at(j)[u].front());
								cur = cur->pnext_;
							} while (cur != sta);
						}
						glEnd();

					}
					else
					{
						glBegin(GL_LINES);
						for (int v = 0; v < sup_lines->at(j)[u].size() - 1; v++)
						{
							glVertex3fv(sup_lines->at(j)[u][v]);
							glVertex3fv(sup_lines->at(j)[u][v + 1]);

						}
						glEnd();
					}
				}
			}
			
			
// 			//glColor4ub(55, 126, 184, 255);
// 			auto sup_rec = ctn_obj[i]->ppcs->su->get_suprec();
// 			for (int u = 0; u < sup_rec->size(); u++)
// 			{
// 				for (int v = 0; v < sup_rec->at(u).size(); v++)
// 				{
// 					glBegin(GL_LINES);
// 					for (int w = 0; w < sup_rec->at(u)[v].size() ; w++)
// 					{
// 						glVertex3fv(sup_rec->at(u)[v][w]);
// 						glVertex3fv(sup_rec->at(u)[v][(w + 1)%sup_rec->at(u)[v].size()]);
// 					}
// 					glEnd();
// 				}
// 
// 			}
			
			glColor4ub(152, 78, 163, 255);
			auto sup_region = ctn_obj[i]->ppcs->su->get_minkowssum();
			
			for (int j=0;j<sup_region->at(slice_check_id_).size();j++)
			{
				glBegin(GL_LINE_LOOP);
				for (int k = 0; k < sup_region->at(slice_check_id_)[j].size(); k++)
				{
					glVertex3fv(sup_region->at(slice_check_id_)[j][k]);
				}
				glEnd();
			}
			
		
			glColor4ub(77, 175, 74, 255);
			
			auto polylines = ctn_obj[i]->ppcs->su->get_polylines();
			for (int u = 0; u < polylines->size(); u++)
			{
				for (int v = 0; v < polylines->at(u).size(); v++)
				{	
					glLineWidth(2.0);
					glBegin(GL_LINES);
					for (int w = 0; w < polylines->at(u)[v].size() - 1; w++)
					{
						glVertex3fv(polylines->at(u)[v][w]);
						glVertex3fv(polylines->at(u)[v][w + 1]);
					}
					glEnd();
				}

			}
			auto sup_hatch= ctn_obj[i]->ppcs->su->get_hatchs();
			glLineWidth(2.0);
			glBegin(GL_LINES);
			for (int j=0;j<sup_hatch->at(slice_check_id_).size();j++)
			{
				glVertex3fv(sup_hatch->at(slice_check_id_)[j].get_v1());
				glVertex3fv(sup_hatch->at(slice_check_id_)[j].get_v2());			
			}
			glEnd();
		}
	}

}
void RenderingWidget::DrawSupFace(bool bv)
{
	if (!bv || ctn_obj.empty())
		return;	
	glLineWidth(5.0);
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{

		std::vector<std::vector<HE_edge*>>& bl = ctn_obj[id_obj]->ptr_support_->wall_mesh.GetBLoop();
		for (size_t i = 0; i != bl.size(); i++)
		{
		
			if (i!=slice_check_id_)
			{
				continue;
			}
			Path obj;
			Paths res;
			glBegin(GL_LINE_LOOP);
			glColor3f(0.0, 0.0, 0.0);
			for (int j = 0; j < bl[i].size(); j++)
			{
				glVertex3f(bl[i][j]->pvert_->position().x(), bl[i][j]->pvert_->position().y(), 0);
				obj << IntPoint(bl[i][j]->pvert_->position().x()*1e3, bl[i][j]->pvert_->position().y()*1e3);
			}
			glEnd();

			ClipperOffset  sov;
			glColor3f(1.0, 0.0, 0.0);
			sov.AddPath(obj, jtMiter, etClosedPolygon);
			sov.Execute(res, -2*1e3);
			for (int m = 0; m < res.size(); m++)
			{
				glBegin(GL_LINE_LOOP);
				for (int n = 0; n < res[m].size(); n++)
				{
					glVertex3f(res[m][n].X/1e3,res[m][n].Y/1e3,0);
				}
				glEnd();
			}
			


			continue;
			auto faces=ctn_obj[id_obj]->ptr_support_->wall_mesh.get_faces_list();
			glColor4ub(170, 0, 0, 255);
			
			glColor3f(1.0, 1.0, 1.0);
		
			for (int j = 0; j < bl[i].size(); j++)
			{
				glBegin(GL_POLYGON);
				glVertex3f(bl[i][j]->pvert_->position().x(), bl[i][j]->pvert_->position().y(), bl[i][j]->pvert_->position().z());
// 				Vec3f p1, p2;
// 				ctn_obj[id_obj]->ptr_support_->GetMeshInOctree()->hitOctreeNode(this->ctn_obj[0]->ptr_support_->GetMeshInOctree()->oc_root_, bl[i][j]->pvert_->position(),
// 					p1, Vec3f(0, 0, -1), hitID, temp_t);
// 				ctn_obj[id_obj]->ptr_support_->GetMeshInOctree()->hitOctreeNode(this->ctn_obj[0]->ptr_support_->GetMeshInOctree()->oc_root_, bl[i][(j + 1) % bl[i].size()]->pvert_->position(),
// 					p2, Vec3f(0, 0, -1), hitID, temp_t);

				glVertex3fv(bl[i][j]->pvert_->position()-Vec3f(0,0,10));
				glVertex3fv(bl[i][(j + 1) % bl[i].size()]->pvert_->position()-Vec3f(0,0,10));
				glVertex3fv(bl[i][(j + 1) % bl[i].size()]->pvert_->position());
				glEnd();
			}
		
			for (int j=0;j<faces->size();j++)
			{
				if (faces->at(i)->com_flag==i)
				{
					HE_edge* sta = faces->at(i)->pedge_;
					HE_edge* cur = sta;
					do 
					{
						glVertex3fv(cur->pvert_->position());
						cur = cur->pnext_;
					} while (cur!=sta);
				}
			}
			glEnd();
		}

	}
}

void RenderingWidget::DrawGrid(bool bv)
{
	if (!bv)
		return;
	glLineWidth(1.0);
	glColor3f(0.9, 0.9, 0.9);
	glBegin(GL_LINES);
	for (int i = -200; i < 201; i+=10)
	{
		glVertex2f(-200, -i);
		glVertex2f(200, -i);
		glVertex2f(i, -200);
		glVertex2f(i, 200);
	}

	glEnd();

	//glEnable(GL_LIGHTING);
}
void RenderingWidget::DrawCutPieces(bool bv)
{

	if (!bv || ctn_obj.empty())
		return;
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		if (ctn_obj[id_obj]->ppcs!=NULL)
		{
			glColor3f(0.0, 0.0, 0.0);
			std::vector<std::vector<std::vector<Segment*>>>* cnts = ctn_obj[id_obj]->ppcs->sl->get_contours();
			for (int i = 1; i <cnts->size(); i++)
			{
				if (i!=slice_check_id_/*&& i!=slice_check_id_+1*/)
				{
					continue;
				}
				glLineWidth(2.0);
				glBegin(GL_LINES);
				for (int j = 0; j < cnts->at(i).size(); j++)
				{
					for (int k = 0; k < cnts->at(i)[j].size(); k++)
					{
						glVertex3fv(cnts->at(i)[j][k]->get_v1());
						glVertex3fv(cnts->at(i)[j][k]->get_v2());
					}
				}
				glEnd();
			}
		}
	}
}
void RenderingWidget::DrawCutPiecesSup(bool bv)
{
	if (!bv || ctn_obj.empty())
		return;
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		if (ctn_obj[id_obj]->ptr_support_==NULL|| ctn_obj[id_obj]->mycutsup==NULL)
		{
			continue;
		}
		std::vector < std::vector<cutLine>* >*tc = (ctn_obj[id_obj]->mycutsup->GetPieces());
		glColor3f(1.0, 0.0, 0.0);
		//for (int i = 0; i<mycut->num_pieces_; i++)
		if (slice_check_id_ >= ctn_obj[id_obj]->mycutsup->GetNumPieces())
		{
			return;
		}
		for (int i = slice_check_id_; i < slice_check_id_ + 1; i++)
		{
			glBegin(GL_LINES);
			for (size_t j = 0; j < tc[i].size(); j++)
			{
				for (int k = 0; k < (tc[i])[j]->size(); k++)
				{
					glVertex3fv((((tc[i])[j])->at(k).position_vert[0] * scaleV).data());
					glVertex3fv((((tc[i])[j])->at(k).position_vert[1] * scaleV).data());
				}

			}
			glEnd();
		}
	}
}

void RenderingWidget::DrawHatch(bool bv)
{
	if (!bv || ctn_obj.empty())
		return;
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		//DrawCutPieces(bv);
		if (ctn_obj[id_obj]->myhatch == NULL)
		{
			return;
		}
		std::vector<Vec3f*>* tc_hatch_ = ctn_obj[id_obj]->myhatch->getHatch();
		std::vector < std::vector<Vec3f>>* tc_offset_ = ctn_obj[id_obj]->myhatch->getOffsetVertex();
		if (slice_check_id_ > ctn_obj[id_obj]->myhatch->GetNumPieces() - 1)
		{
			return;
		}
		if (is_show_all)
		{
			for (int i = 0; i < ctn_obj[id_obj]->myhatch->GetNumPieces(); i += 100)
			{
				for (auto iterline = tc_hatch_[i].begin(); iterline != tc_hatch_[i].end(); iterline++)
				{
					glColor3f(0.0, 1.0, 0.0);
					glBegin(GL_LINES);
					glVertex3fv(((*iterline)[0] * scaleV));
					glVertex3fv(((*iterline)[1] * scaleV));
					glEnd();
				}
				for (int j = 0; j < tc_offset_[i].size(); j++)
				{
					glColor3f(0.0, 0.0, 0.0);
					glBegin(GL_LINE_LOOP);
					for (int k = 0; k < ((tc_offset_[i])[j]).size(); k++)
					{
						glVertex3fv((((tc_offset_[i])[j]).at(k)*scaleV).data());
					}
					glEnd();
				}
			}
		}
		else
		{
			for (int i = slice_check_id_; i < slice_check_id_ + 1; i++)
			{
				for (auto iterline = tc_hatch_[i].begin(); iterline != tc_hatch_[i].end(); iterline++)
				{
					glLineWidth(2.0);
					glColor3f(1.0, 1.0, 1.0);
					glBegin(GL_LINES);
					glVertex3fv(((*iterline)[0] * scaleV));
					glVertex3fv(((*iterline)[1] * scaleV));
					glEnd();
				}
				//continue;
				for (int j = 0; j < tc_offset_[i].size(); j++)
				{
					glColor3f(0.0, 0.0, 0.0);
					//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
					glBegin(GL_LINE_LOOP);
					for (int k = 0; k < ((tc_offset_[i])[j]).size(); k++)
					{
						//qDebug() << ((tc_offset_[i])[j])->at(k).x() << ((tc_offset_[i])[j])->at(k).y() << ((tc_offset_[i])[j])->at(k).z();
						glVertex3fv((((tc_offset_[i])[j]).at(k)*scaleV).data());
					}
					glEnd();
				}

			}
		}
	}
}

void RenderingWidget::ClearSlice()
{
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		if (mycut != NULL)
		{
			mycut->clearcut();
			mycut = NULL;
		}
		if (mycutsup != NULL)
		{
			mycutsup->clearcut();
			mycutsup = NULL;
		}
	}
}
void RenderingWidget::ClearHatch()
{
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		if (myhatch != NULL)
		{
			myhatch->clearHatch();
			myhatch = NULL;
		}
		if (myhatchsup != NULL)
		{
			myhatchsup->clearHatch();
			myhatchsup = NULL;
		}
	}
}
void RenderingWidget::ClearSupport()
{
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		ptr_support_->GetMeshSupport()->ClearData();
	}
}


// support operators
void RenderingWidget::AddPointSupport() {
	isAddPoint = !isAddPoint;
}
void RenderingWidget::AddLineSupport() {
	isAddLine = !isAddLine;

}
void RenderingWidget::DeleteSupport() {
	isDelete = !isDelete;
}

void RenderingWidget::AutoSupport()
{
	for (int i = 0; i < ctn_obj.size(); i++)
	{
		if (ctn_obj[i]->ppcs==NULL)
		{
			ctn_obj[i]->ppcs = new Preprocessor(ctn_obj[i]->ptr_mesh_);
		}
		ctn_obj[i]->ppcs->add_support();
	}
	update();
}

void RenderingWidget::setPointD(double diameter)
{
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

		ptr_support_->SetPoint(diameter, 0.0);
	}
}
void RenderingWidget::setPointH(double diameter)
{
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

		ptr_support_->SetPoint(0.0, diameter);

	}
}
void RenderingWidget::setLineD(double diameter)
{
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

		ptr_support_->SetLine(diameter, 0.0);
	}
}
void RenderingWidget::setLineH(double diameter)
{
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

		ptr_support_->SetLine(0.0, diameter);
	}
}

//hatch operators
void RenderingWidget::objectTransformation(float * matrix)
{
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

		ptr_mesh_->Transformation(matrix);
		ptr_support_->GetMeshSupport()->Transformation(matrix);
	}
}
void RenderingWidget::Translation()
{
}
void RenderingWidget::SetDirection(int id)
{
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

		//is_select_face = false;
		ptr_mesh_->SetDirection(id);
	}
}
void RenderingWidget::cutinPieces()
{

}
void RenderingWidget::cutinPiecesSup()
{
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

		if (ptr_support_->GetMeshSupport()->num_of_vertex_list() == 0)
		{
			return;
		}
		if (mycutsup != NULL)
		{
			mycutsup->~SliceCut();
		}
		//ptr_support_->GetMeshSupport()->UpdateMesh();
		mycutsup = new SliceCut(ptr_support_->GetMeshSupport());
		//isAddLine = true;
		is_draw_face_ = false;
		//is_draw_point_ = false;
		is_draw_edge_ = false;
		//is_draw_cutpieces_ = !is_draw_cutpieces_;
		mycutsup->clearcut();
		mycutsup->storeMeshIntoSlice();
		mycutsup->CutInPieces();
		//Export();
	}
	update();
}

void RenderingWidget::SelectFace(int x, int y)
{
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

		OpenGLProjector myProjector = OpenGLProjector();

		Vec3f u(x, y, 0.0);
		qDebug() << "u:" << u.x() << u.y() << u.z();
		const std::vector<HE_face *>& faces = *(ptr_mesh_->get_faces_list());

		double mindis = 1e6; int selectedFacet = -1;
		for (size_t i = 0; i < faces.size(); i++)
		{
			//qDebug() << "*****************" ;
			Vec3f v = myProjector.Project(faces[i]->center());

			//qDebug() << myProjector.GetDepthValue((int)v.x(), (int)v.y());
			//qDebug() << myProjector.GetDepthValue((int)v.x(), (int)v.y()) - v.z();
			if (myProjector.GetDepthValue((int)v.x(), (int)v.y()) - v.z() <= -1e-2)
			{
				continue;
			}
			//qDebug() << "v:"<<v.x() << v.y() << v.z();
			v.z() = 0;
			double dist = (u - v).length();
			if (dist < mindis)
			{
				mindis = dist;
				selectedFacet = (int)i;
			}
		}
		if (selectedFacet != -1)
		{
			current_face_ = selectedFacet;

			qDebug() << current_face_;
		}
	}
}
void RenderingWidget::renderdoHatch()
{
	for (int i = 0; i < ctn_obj.size(); i++)
	{
		if (ctn_obj[i]->ppcs == NULL)
			ctn_obj[i]->ppcs = new Preprocessor(ctn_obj[i]->ptr_mesh_);
		ctn_obj[i]->ppcs->do_slice();
	}
	update();
}

//set varible
void RenderingWidget::setfieldWidth(double width)
{
	field_width_ = width;
}
void RenderingWidget::setfieldHeight(double height)
{
	field_height_ = height;
}
void RenderingWidget::setlineOverlap(int lineoverlap)
{
	line_width_ = lineoverlap / 1000;
}
void RenderingWidget::setfieldOverlap(double fieldoverlap)
{
	field_overlap_ = fieldoverlap;
}
void RenderingWidget::setHatchDis(double value)
{
	line_width_ = value;
}
void RenderingWidget::setThickness(double thick)
{
	thickness_ = thick;
}
void RenderingWidget::setAngle(int angle)
{
	increment_angle_ = angle;
}
void RenderingWidget::setLineError(double err) {
	ERR = err; 
}
void RenderingWidget::FindRegion()
{
	is_draw_region_ = !is_draw_region_;
	if (!is_draw_region_)
	{
		parent->ui.pushButton_3->setChecked(is_draw_region_);
		parent->ui.pushButton->setChecked(is_draw_region_);
		update();
		return;
	}
	parent->ui.pushButton_3->setChecked(is_draw_region_);
	parent->ui.pushButton->setChecked(is_draw_region_);

	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

		std::vector<HE_face*>* faceList = ptr_mesh_->get_faces_list();
		int faceNum = faceList->size();
		std::vector<HE_edge*>* edgeList = ptr_mesh_->get_edges_list();
		int edgeNum = ptr_mesh_->get_edges_list()->size();
		for (int i=0;i<faceNum;i++)
		{
			std::vector<HE_vert *> tmp_face_verts;
			faceList->at(i)->face_verts(tmp_face_verts);

			Vec3f eAB = tmp_face_verts[1]->position_ - tmp_face_verts[0]->position_;
			Vec3f eAC = tmp_face_verts[2]->position_ - tmp_face_verts[0]->position_;
			Vec3f vNormal = eAB.cross(eAC);
			vNormal.normalize();

			float angle = vNormal.dot(Vec3f(0,0,-1));
		}

	}


	update();
}
//by Triangle Maintenancer
//Reset Model View
void RenderingWidget::ResetView() {

	ptr_arcball_->InitBall();

	update();
}

//pass information for SendMsg 
void RenderingWidget::RecvMsg(QString str) {
	for (int id_obj = 0; id_obj < ctn_obj.size(); id_obj++)
	{
		SliceCut* mycut = ctn_obj[id_obj]->mycut;
		SliceCut* mycutsup = ctn_obj[id_obj]->mycutsup;
		Hatch* myhatch = ctn_obj[id_obj]->myhatch;
		Hatch* myhatchsup = ctn_obj[id_obj]->myhatchsup;
		Support* ptr_support_ = ctn_obj[id_obj]->ptr_support_;
		Mesh3D* ptr_mesh_ = ctn_obj[id_obj]->ptr_mesh_;

		if (str == "intersectDetection")
		{
			//执行检测方法
			ptr_mesh_->TriangleIntersect();
			//ptr_mesh_->ClearMark();
			emit sendMsgtoDialog("Dialog");
		}
		else if (str == "reverseDetection")
		{
			//2017/10/23还未做reverse处理
			emit sendMsgtoDialog("Dialog");
		}
		else if (str == "badsideDetection")
		{
			//坏边数量即bhelist
			badside_num = ptr_mesh_->GetBhelist()->size();
			emit sendMsgtoDialog("Dialog");
		}
		else if (str == "holeDetection")
		{
			//孔洞数量即bloop
			hole_num = ptr_mesh_->GetBLoop().size();
			emit sendMsgtoDialog("Dialog");
		}
		else if (str == "shellDetection")
		{
			//壳体的检测在Mesh.cpp中updateMesh中的setBloopFromBhelist已经做了处理
			emit sendMsgtoDialog("Dialog");
		}
		else if (str == "update")
		{
			//做一些重置更新工作，在点击更新后，可能做了一些修复，未来的及更新Tria

		}
		else if (str == "maintenance")
		{
			ptr_mesh_->Maintenance();
		}

		else if (str == "clearmark")
		{
			ptr_mesh_->ClearMark();
		}

		else if (str == "intersectmark")
		{
			ptr_mesh_->TriaToTri();
		}

		else if (str == "createtriangle")
		{
			//开放openGLWidget的选取，
			ptr_mesh_->CreateTriangle();
		}

		else if (str == "repairhole")
		{	//孔洞的修复
			ptr_mesh_->RepairHole();
		}

		else if (str == "markbadside")
		{
			badsideMark = !badsideMark;
		}
		//else
		//{
		//	updateFlag = true;
		//}

		//设置dialog上界面变化
	}
	update();
}

void RenderingWidget::ApplyMaintenance() {
	qDebug() << "Maintenance!!!" << "\n";
	if (actionMaintenanceFlag)
	{
		actionMaintenanceFlag = false;

		MyDialog *dialog = new MyDialog(this);//当用WA_DeleteOnClose时，会不会存在二次析构的问题呢，不会，对象析构时，会在父对象的children table delete.父对象销毁时children也销毁，否则nodestory
		dialog->setAttribute(Qt::WA_DeleteOnClose);//在Dialog关闭时，进行析构，防止内存泄漏
		dialog->setWindowTitle(tr("Maintenance Dialog"));
		//关联信号和槽函数
		connect(dialog, SIGNAL(SendMsg(QString)), this, SLOT(RecvMsg(QString)));
		connect(this, SIGNAL(sendMsgtoDialog(QString)), dialog, SLOT(receiveData(QString)));

		dialog->show();
		qDebug() << "over!!!" << dialog->result();
	}
}  


