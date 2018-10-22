#pragma once
#include "HE_mesh/Vec.h"
#include "HE_mesh/Mesh3D.h"
class segment
{
public:
protected:
private:
	Vec3f v1, v2;
	float angle;
};
class slicer
{
private:
	Mesh3D* obj;
	float thickness;
	int num;
	std::vector<std::vector<std::vector<segment>>> contours;
public:
	slicer();
	~slicer();
	slicer(Mesh3D* tar,float in_thickness=0.09);
	void execute();
};

