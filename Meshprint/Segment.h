#pragma once
#include "HE_mesh/Mesh3D.h"
class Segment
{

protected:
private:
	Vec3f v1, v2;
	float angle;
	bool is_vailed;
public:
	void set_v1(Vec3f v) { v1 = v; };
	void set_v2(Vec3f v) { v2 = v; };
	void set_angle(float a) { angle = a; };
	bool vailed() { return  v1 != v2; };
	Vec3f get_v1() { return v1; };
	Vec3f get_v2() { return v2; };
};
