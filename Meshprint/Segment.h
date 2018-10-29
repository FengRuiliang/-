#pragma once
#include "HE_mesh/Mesh3D.h"
class Segment
{

protected:
private:
	Vec3f v1, v2;// the two endpoint of this segment
	Vec2f vu, vl;// the upper point and the lower point of this segment
	float angle;//for add support point for FDM, has nothing business with sweep line algorithm
	Vec3f vout, vin;
	//the follow variable is used for sweep algorithm
	Vec2f pc;
	bool toereas=false;
	bool belong_u_and_c = false;
	bool is_vailed;//judge whether this segment can be used for sweep line algorithm
	//
public:
	void set_v1(Vec3f v) { v1 = v; };
	void set_v2(Vec3f v) { v2 = v; };
	void set_pc(Vec3f v) { pc.x() = v.x(); pc.y() = v.y(); }
	void set_vu(Vec3f v) { vu.x() = v.x(); vu.y() = v.y(); }
	void set_vl(Vec3f v) { vl.x() = v.x(); vl.y() = v.y(); }
	void set_vout(Vec3f v) { vout = v; }
	void set_vin(Vec3f v) { vin= v; }
	void set_angle(float a) { angle = a; };
	bool vailed() { return  v1 != v2; };
	Vec3f &get_v1() { return v1; };
	Vec3f &get_v2() { return v2; };
	float get_angle() { return angle; }
	// the following codes is used for sweep algorithm
	Vec2f &get_pc() { return pc; }
	Vec2f &get_vu() { return vu; }
	Vec2f &get_vl() { return vl; }
	bool set_to_erease(bool param1) { return toereas = param1; }
	bool get_to_erease() { return toereas; }

};
