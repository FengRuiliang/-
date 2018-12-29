#include "Preprocessor.h"
#include<fstream>
#include <cmath>

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
	qDebug() << "successfully add supporting point";
	exportpoint();
	exportline();
	int num = 0;
	for (int i=0;i<su->get_sup_points()->size();i++)
	{
		num += su->get_sup_points()->at(i).size();
		if (su->get_sup_points()->at(i).size() != 0)
		{
			qDebug() << "i" << i << "#" << su->get_sup_points()->at(i).size();
		}

	}
	qDebug() <<"#:"<< num;
}
void Preprocessor::exportpoint()
{
	auto ptr_points = su->get_sup_points();
	std::ofstream fout("D:/grs/a.grs");

	fout << "ENTITY/OBJ" << std::endl;
	double a[3], b[3], diaa, diab, axis[3], dis, dia=0.6;
	for (int i = 0; i < ptr_points->size(); i++)
	{
		for each(Vec3f p in ptr_points->at(i))
		{
			a[0] = p.x();
			a[1] = p.y();
			a[2] = p.z();
			diaa = 1.0;
			diab = 1.1;

			b[0] = a[0];
			b[1] = a[1];
			b[2] = 0;
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
