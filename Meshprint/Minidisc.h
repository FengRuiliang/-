#pragma once
#include <utility>
#include "HE_mesh\Mesh3D.h"
namespace MINIDISC {
	std::pair<Vec2f, float> miniDiscWith2Point(std::vector<Vec2f> P, Vec2f q1, Vec2f q2);
	std::pair<Vec2f, float> miniDiscWithPoint(std::vector<Vec2f> P, Vec2f q);
	std::pair<Vec2f, float> miniDisc(std::vector<Vec2f> pointsin);
}