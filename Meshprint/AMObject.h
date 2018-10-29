#pragma once
#include "globalFunctions.h"
#include <QEvent>
#include "HE_mesh/Vec.h"
#include "Hatch.h"
#include "Segment.h"
#include "Preprocessor.h"
class Meshprint;
class CArcBall;
class Mesh3D;
class Support;
class SliceCut;
class Hatch;
class HatchChessboard;
enum hatchType
{
	NONE = 0,
	CHESSBOARD,
	OFFSETFILLING,
	X_PARALLEL,
	Y_PARALLEL,
	HYBRID,

};
using trimesh::vec;
using trimesh::point;
typedef trimesh::vec3  Vec3f;
class AMObject
{
public:
	Mesh3D						*ptr_mesh_;
	Preprocessor					* ppcs;
	SliceCut* mycut;
	SliceCut*	mycutsup;
	Hatch						*myhatch;
	Hatch						*myhatchsup;
	//hatch operator
	int					hatch_type_;
	// my Support-operator
	//Mesh3D						*ptr_mesh_support_;
	Support						*ptr_support_;

	// Texture
public:
	AMObject();
	~AMObject();
};

