#include "Preprocessor.h"
#include<fstream>
#include <cmath>
#include "Support.h"

void Preprocessor::exportToUG(std::vector<std::vector<std::vector<Vec3f>>>* lines)
{
	MeshOctree oct;
	oct.BuildOctree(tar);
	std::ofstream fout("D:/OneDrive/бшнд/ugpart/a.grs");
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
			else
			{ 
				for (int k = 0; k < lines->at(i)[j].size() - 1; k++)
				{
					fout << "LINES(1)=line/"
						<< lines->at(i)[j][k].x() << "," << lines->at(i)[j][k].y() << "," << i*thickness_
						<< "," << lines->at(i)[j][k].x() << "," << lines->at(i)[j][k].y() << "," << lines->at(i)[j][k].z() << std::endl;
					fout << "LINES(2)=line/" 
						 << lines->at(i)[j][k].x() << "," << lines->at(i)[j][k].y() << "," << lines->at(i)[j][k].z() 
						<< "," << lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," << lines->at(i)[j][k + 1].z()  << std::endl;
					fout << "LINES(3)=line/" 
						 << lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," << lines->at(i)[j][k + 1].z()
						<< "," << lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," << i*thickness_ << std::endl;
					fout << "LINES(4)=line/"
						<< lines->at(i)[j][k + 1].x() << "," << lines->at(i)[j][k + 1].y() << "," << i*thickness_
						<< "," << lines->at(i)[j][k].x() <<  ","<<lines->at(i)[j][k].y() << "," << i*thickness_ << std::endl;
					fout << "OBJ=BPLANE/LINES" << std::endl;
				}
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
	su->add_supportting_point_for_contours(sl->get_contours());

	exportToUG(su->get_suplines());
	//exportpoint();
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
	double a[3], b[3], diaa, diab, axis[3], dis, dia=0.6;
	//int j = 0;
	//for (Vec3f p : ptr_points->at(80))
	//{
	//	Vec3f point_out = oct.InteractPoint(p, Vec3f(0, 0, -1));
	//	a[0] = p.x();
	//	a[1] = p.y();
	//	a[2] = p.z();
	//	b[0] = point_out.x();
	//	b[1] = point_out.y();
	//	b[2] = point_out.z();
	//	axis[0] = b[0] - a[0];
	//	axis[1] = b[1] - a[1];
	//	axis[2] = b[2] - a[2];

	//	dis = (point_out - p).length();
	//	fout << "OBJ=SOLCYL/ORIGIN," << a[0] << "," << a[1] << "," << a[2] << ",HEIGHT,$" << std::endl
	//		<< dis << ",DIAMTR," << dia << ",AXIS," << axis[0] << "," << axis[1] << "," << axis[2] << std::endl;

	//}
	//float tem = 0;
	//for (Vec3f p:ptr_points->at(465))
	//{
	//	Vec3f point_out = oct.InteractPoint(p, Vec3f(0, 0, -1));
	//	a[0] = p.x();
	//	a[1] = p.y();
	//	a[2] = p.z();
	//	b[0] = point_out.x();
	//	b[1] = point_out.y();
	//	b[2] = point_out.z();
	//	axis[0] = b[0] - a[0];
	//	axis[1] = b[1] - a[1];
	//	axis[2] = b[2] - a[2];
	//	dia = 0.6;
	//	dis = (point_out - p).length()/3;
	//	fout << "OBJ=SOLCYL/ORIGIN," << a[0] << "," << a[1] << "," << a[2] << ",HEIGHT,$" << std::endl
	//		<< dis << ",DIAMTR," << dia << ",AXIS," << axis[0] << "," << axis[1] << "," << axis[2] << std::endl;
	//	fout << "OBJ=SOLCYL/ORIGIN," << a[0] << "," << a[1] << "," << a[2]-dis+0.1 << ",HEIGHT,$" << std::endl
	//		<< dis*2 << ",DIAMTR," << 1.0 << ",AXIS," << axis[0] << "," << axis[1] << "," << axis[2] << std::endl;
	//	tem = a[2] - dis;
	//}
	//
	//for (Vec3f p : ptr_points->at(460))
	//{
	//	dia = 0.6;
	//	a[0] = p.x();
	//	a[1] = p.y();
	//	a[2] = p.z();
	//	axis[0] = b[0] - a[0];
	//	axis[1] = b[1] - a[1];
	//	axis[2] = tem - a[2];

	//	
	//	dis = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
	//	fout << "OBJ=SOLCYL/ORIGIN," << a[0] << "," << a[1] << "," << a[2] << ",HEIGHT,$" << std::endl
	//		<< dis << ",DIAMTR," << dia << ",AXIS," << axis[0] << "," << axis[1] << "," << axis[2] << std::endl;


	for (int i = 0; i < ptr_points->size(); i++)
	{
		for each(Vec3f p in ptr_points->at(i))
		{
			Vec3f point_out = oct.InteractPoint(p, Vec3f(0, 0, -1));
			a[0] = p.x();
			a[1] = p.y();
			a[2] = p.z();
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

			//fout << "CONE=SOLCON/ORIGIN," << a[0] << "," << a[1] << "," << a[2] << ",HEIGHT,$" << std::endl;
			//fout << dis << ",DIAMTR," << diaa << "," << diab << ",AXIS," << axis[0] << "," << axis[1] << "," << axis[2] << std::endl;
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
