#include "Preprocessor.h"
#include<fstream>
#include <cmath>
#include "Support.h"

void Preprocessor::exportToUG(std::vector<std::vector<std::vector<Vec3f>>>* lines)
{
	MeshOctree oct;
	oct.BuildOctree(tar);
	std::ofstream fout("D:/grs/a.grs");
	fout << "ENTITY/OBJ,LINES(4)" << std::endl;
	for (int i = 0; i < lines->size(); i++)
	{
		for (int j = 0; j < lines->at(i).size(); j++)
		{
			if (lines->at(i)[j].size() == 1)
			{
				
				Vec3f point_in=lines->at(i)[j].front();
				Vec3f point_out = oct.InteractPoint(point_in, Vec3f(0, 0, -1));
				Vec3f axis = point_out - point_in;
				axis.normalize();
				fout << "OBJ=SOLCYL/ORIGIN," 
					<< point_in.x() << "," << point_in.y() << "," << point_in.z()
					<< ",HEIGHT,"<< (point_in-point_out).length() 
					<< ",DIAMTR," << 1.0 
					<< ",AXIS," << axis[0] << "," << axis[1] << "," << axis[2] 
					<< std::endl;
			}
			else if ((lines->at(i)[j].size() <4))
			{ 
				for (int k = 0; k < lines->at(i)[j].size() - 1; k++)
				{
					float height = i*thickness_;
					fout << "LINES(1)=line/"
						<< lines->at(i)[j][k].x() << "," << lines->at(i)[j][k].y() << "," <<height
						<< "," << lines->at(i)[j][k].x() << "," << lines->at(i)[j][k].y() << "," << lines->at(i)[j][k].z() << std::endl;
					fout << "LINES(2)=line/" 
						 << lines->at(i)[j][k].x() << "," << lines->at(i)[j][k].y() << "," << lines->at(i)[j][k].z() 
						<< "," << lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," << lines->at(i)[j][k + 1].z()  << std::endl;
					fout << "LINES(3)=line/" 
						 << lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," << lines->at(i)[j][k + 1].z()
						<< "," << lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," <<height << std::endl;
					fout << "LINES(4)=line/"
						<< lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," <<height
						<< "," << lines->at(i)[j][k].x() <<  ","<<lines->at(i)[j][k].y() << "," <<height << std::endl;
					fout << "OBJ=BPLANE/LINES" << std::endl;
				}
			}
			else
			{

				float height= lines->at(i)[j].back().z() -10*thickness_;
				for (int k = 0; k< lines->at(i)[j].size() - 1; k++)
				{
					fout << "LINES(1)=line/"
						<< lines->at(i)[j][k].x() << "," << lines->at(i)[j][k].y() << "," << height
						<< "," << lines->at(i)[j][k].x() << "," << lines->at(i)[j][k].y() << "," << lines->at(i)[j][k].z() << std::endl;
					fout << "LINES(2)=line/"
						<< lines->at(i)[j][k].x() << "," << lines->at(i)[j][k].y() << "," << lines->at(i)[j][k].z()
						<< "," << lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," << lines->at(i)[j][k + 1].z() << std::endl;
					fout << "LINES(3)=line/"
						<< lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," << lines->at(i)[j][k + 1].z()
						<< "," << lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," << height << std::endl;
					fout << "LINES(4)=line/"
						<< lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," << height
						<< "," << lines->at(i)[j][k].x() << "," << lines->at(i)[j][k].y() << "," << height << std::endl;
					fout << "OBJ=BPLANE/LINES" << std::endl;
				}
				
				Vec3f point_in = lines->at(i)[j].front();
				point_in.z() = height;
				Vec3f point_out = oct.InteractPoint(point_in, Vec3f(0, 0, -1));
				Vec3f axis = point_out - point_in;
				axis.normalize();
				fout << "OBJ=SOLCYL/ORIGIN,"
					<< point_in.x() << "," << point_in.y() << "," << point_in.z()
					<< ",HEIGHT," << (point_in - point_out).length()
					<< ",DIAMTR," << 1.0
					<< ",AXIS," << axis[0] << "," << axis[1] << "," << axis[2]
					<< std::endl;
				point_in = lines->at(i)[j].back();
				point_in.z() = height;
				point_out = oct.InteractPoint(point_in, Vec3f(0, 0, -1));
				axis = point_out - point_in;
				axis.normalize();
				fout << "OBJ=SOLCYL/ORIGIN,"
					<< point_in.x() << "," << point_in.y() << "," << point_in.z()
					<< ",HEIGHT," << (point_in - point_out).length()
					<< ",DIAMTR," << 1.0
					<< ",AXIS," << axis[0] << "," << axis[1] << "," << axis[2]
					<< std::endl;
				
			}
		}
	}
	fout << "halt" << std::endl;
}

Preprocessor::Preprocessor()
{
}


Preprocessor::~Preprocessor()
{
	delete sl;
	delete su;
}

void Preprocessor::do_slice()
{
	if (sl == NULL)
	{
		sl = new Slicer(tar);
		sl->doslice();

	}
}

void Preprocessor::add_support()
{


	do_slice();
	su = new Supportor;
	su->generatePointsAndRibs(sl->get_contours());
	//su->add_supportting_rib_for_contours(sl->get_contours());
	//exportTriangles();
	exportpoint();

	return;
	//exportToUG(su->get_suplines());
	
	//exportline();
	int num = 0;
	auto lines = su->get_suplines();
	for (int i=0;i<lines->size();i++)
	{
		for (int j = 0; j < lines->at(i).size(); j++)
		{
			num += lines->at(i)[j].size();
		}
		if (lines->at(i).size() != 0)
		{
			qDebug() << "i" << i << "#" << lines->at(i).size()<<num;
		}
	}
	qDebug() <<"#:"<< num;
}
void Preprocessor::exportpoint()
{
	MeshOctree oct;
	oct.BuildOctree(tar);
	auto ptr_points = su->get_sup_points();
	std::ofstream fout("D:/grs/a.grs");
	fout << "ENTITY/OBJ" << std::endl;
	double a[3], b[3], diaa, diab, axis[3], dis, dia=1.0;

	for (int i = 0; i < ptr_points->size(); i++)
	{
		for each(Vec3f p in ptr_points->at(i))
		{
			Vec3f point_out = oct.InteractPoint(p, Vec3f(0, 0, -1));
			a[0] = p.x();
			a[1] = p.y();
			a[2] = p.z()-1.0;
			diaa = 1.0;
			diab = 1.1;

			b[0] = point_out.x();
			b[1] = point_out.y();
			b[2] = point_out.z();
			for (int i = 0; i < 3; i++)
			{
				axis[i] = b[i] - a[i];
			}
			dis = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
			fout << "OBJ=SOLCYL/ORIGIN," << a[0] << "," << a[1] << "," << a[2] << ",HEIGHT,$" << std::endl
				<< dis << ",DIAMTR," << dia << ",AXIS," << axis[0] << "," << axis[1] << "," << axis[2] << std::endl;
		}

	}
	fout << "halt";
}
void Preprocessor::exportline()
{
	auto ptr_lines = su->get_suprec();
	std::ofstream fout("D:/grs/b.grs");

	fout << "ENTITY/OBJ,LINES(100)" << std::endl;
	double a[3], b[3], diaa, diab, axis[3], dis, dia = 0.6;
	for (int i = 0; i < ptr_lines->size(); i++)
	{
		for (int j=0;j<ptr_lines->at(i).size();j++)
		{
			/*fout << "ENTITY/LINES(" << ptr_lines->at(i)[j].size()-1 << ")," <<std::endl;*/
			for (int k = 0; k < ptr_lines->at(i)[j].size(); k++)
			{
				fout << "LINES("<< k + 1 <<")=" << "line/"
					<< ptr_lines->at(i)[j][k].x() << ","
					<< ptr_lines->at(i)[j][k].y() << ","
					<< ptr_lines->at(i)[j][k].z() << ","
					<< ptr_lines->at(i)[j][(k + 1)% ptr_lines->at(i)[j].size()].x() << ","
					<< ptr_lines->at(i)[j][(k + 1) % ptr_lines->at(i)[j].size()].y() << ","
					<< ptr_lines->at(i)[j][(k + 1) % ptr_lines->at(i)[j].size()].z()<<std::endl;
			}
			fout << "OBJ=SOLEXT/LINES(1.." << ptr_lines->at(i)[j].size() << "),"
				<< "HEIGHT,"<<0.9<< ",AXIS,"<< 0<<","<<0<<","<<-1<<std::endl;
			fout << "DELETE/LINES(100)" << std::endl;
		}
	}
	fout << "halt";
}
void Preprocessor::exportTriangles()
{
	auto faces = tar->get_faces_list();
	std::ofstream fout("D:/grs/a.grs");
	
	std::vector<bool> is_need(faces->size(), false);
	for (int i = 0; i < faces->size(); i++)
	{
		if (faces->at(i)->center().z() < 10)
		{
			continue;
		}
		if (faces->at(i)->normal().dot(Vec3f(0, 0, -1)) > cos(21 * 3.14 / 180))
		{
			is_need[i] = true;
		}
	}
	fout << "ENTITY/OBJ,TRIANGLE(3),RECTANGLE(4)" << std::endl;
	for (int i = 0; i < faces->size(); i++)
	{
		if (is_need[i])
		{
			
			std::vector<HE_vert*>points;
			faces->at(i)->face_verts(points);
			fout << "TRIANGLE(1)=line/"
				<< points[0]->position().x() << ","
				<< points[0]->position().y() << ","
				<< points[0]->position().z() << ","
				<< points[1]->position().x() << ","
				<< points[1]->position().y() << ","
				<< points[1]->position().z() << std::endl;
			fout << "TRIANGLE(2)=line/"
				<< points[1]->position().x() << ","
				<< points[1]->position().y() << ","
				<< points[1]->position().z() << ","
				<< points[2]->position().x() << ","
				<< points[2]->position().y() << ","
				<< points[2]->position().z() << std::endl;
			fout << "TRIANGLE(3)=line/"
				<< points[2]->position().x() << ","
				<< points[2]->position().y() << ","
				<< points[2]->position().z() << ","
				<< points[0]->position().x() << ","
				<< points[0]->position().y() << ","
				<< points[0]->position().z() << std::endl;
			fout << "OBJ=BPLANE/TRIANGLE" << std::endl;

			fout << "TRIANGLE(1)=line/"
				<< points[0]->position().x() << ","
				<< points[0]->position().y() << ","
				<< points[0]->position().z() - 1.0 << ","
				<< points[1]->position().x() << ","
				<< points[1]->position().y() << ","
				<< points[1]->position().z() - 1.0 << std::endl;
			fout << "TRIANGLE(2)=line/"
				<< points[1]->position().x() << ","
				<< points[1]->position().y() << ","
				<< points[1]->position().z() - 1.0 << ","
				<< points[2]->position().x() << ","
				<< points[2]->position().y() << ","
				<< points[2]->position().z() - 1.0 << std::endl;
			fout << "TRIANGLE(3)=line/"
				<< points[2]->position().x() << ","
				<< points[2]->position().y() << ","
				<< points[2]->position().z() - 1.0 << ","
				<< points[0]->position().x() << ","
				<< points[0]->position().y() << ","
				<< points[0]->position().z() - 1.0 << std::endl;
			fout << "OBJ=BPLANE/TRIANGLE" << std::endl;
			HE_edge* sta = faces->at(i)->pedge_;
			HE_edge* cur = sta;
			do 
			{
				if (!is_need[cur->ppair_->pface_->id()])
				{
					
					Vec3f p1=cur->pprev_->pvert_->position();
					Vec3f p2 = cur->pvert_->position();
					fout << "RECTANGLE(1)=line/"
						<< p1.x() << ","
						<< p1.y() << ","
						<< p1.z() << ","
						<< p2.x() << ","
						<< p2.y() << ","
						<< p2.z() << std::endl;
					fout << "RECTANGLE(2)=line/"
						<< p2.x() << ","
						<< p2.y() << ","
						<< p2.z() << ","
						<< p2.x() << ","
						<< p2.y() << ","
						<< p2.z()-1.0 << std::endl;
					fout << "RECTANGLE(3)=line/"
						<< p2.x() << ","
						<< p2.y() << ","
						<< p2.z()-1.0 << ","
						<< p1.x() << ","
						<< p1.y() << ","
						<< p1.z() - 1.0 << std::endl;
					fout << "RECTANGLE(4)=line/"
						<< p1.x() << ","
						<< p1.y() << ","
						<< p1.z() - 1.0 << ","
						<< p1.x() << ","
						<< p1.y() << ","
						<< p1.z() << std::endl;
					fout << "OBJ=BPLANE/RECTANGLE" << std::endl;
				}
				cur = cur->pnext_;
			} while (cur!=sta);
		}
	}
	fout << "halt";

}