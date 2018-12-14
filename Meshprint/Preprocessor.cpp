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
if (sl==NULL)
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
	exportp();
	qDebug() <<"zhichengdian geshu:"<< su->get_sup_points()->size();
}
void Preprocessor::exportp()
{
	std::ofstream fout("D:/grs/sm_area.grs");

	fout << "ENTITY/CONE" << std::endl;
	double a[3], b[3], diaa, diab, axis[3], dis;
	for each(Vec3f p in *(su->get_sup_points()))
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
		fout << "CONE=SOLCON/ORIGIN," << a[0] << "," << a[1] << "," << a[2] << ",HEIGHT,$" << std::endl;
		fout << dis << ",DIAMTR," << diaa << "," << diab << ",AXIS," << axis[0] << "," << axis[1] << "," << axis[2] << std::endl;
	}
	fout << "halt";
}
