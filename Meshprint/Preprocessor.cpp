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
	su->opitimizePointsandRibs();
	return;
}
