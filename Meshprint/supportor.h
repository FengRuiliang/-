#pragma once
#include <vector>
#include "Segment.h"
#include "Sweep.h"
#include "slicer.h"
#include "clipper.hpp"
#include "Hatch.h"
#include "Minidisc.h"
using namespace ClipperLib;
class Rib;
class Node
{
private:
	Vec3f pos;
	Rib * rib;
	HE_edge* edge_;
	Vec3f normal_;
	Segment* seg_;
public:
	Node(Vec3f in_) { 
		pos = in_;
		rib = NULL;
	};
	~Node() {};
	Node(Segment* in_)
	{
		pos = in_->get_v2();
		seg_ = in_;
		rib = NULL;
		edge_ = in_->get_edge();
		normal_ = seg_->get_normal();
	};
	HE_edge* Edge() { return edge_; }
	Vec3f&    getNormal() { return normal_; }
	Rib*	getRib() { return rib; }
	Segment* getSegments() { return seg_; }
	Vec3f getPosition() { return pos; }
	void setRib(Rib* rib_in_)
	{
		rib = rib_in_;
	};
};
class Rib
{
public:
	Rib(Node* ni_) 
	{
		edge_ = ni_->Edge();

		normal_ = ni_->getNormal();
		candidate_node = NULL;
		angle_ = -1;
	};
	~Rib() {};
	HE_edge* Edge() { return edge_; }
	Vec3f& Normal() { return normal_; }
	Node* CandidateNode() { return candidate_node; }
	std::vector<Node*>& Nodes() { return nodes; }
	float Angle() { return angle_; }
	void setAngle(float in) { angle_ = in; }
	void setCandidateNode(Node* node_in_) { candidate_node = node_in_; }
	void setEdge(HE_edge* in_) { edge_ = in_; };
private:
	HE_edge* edge_;
	HE_edge* candidate_edge_;
	Vec3f normal_;
	Node* candidate_node;
	std::vector<Node*> nodes;
	float angle_;
};
class Polyline
{
public:
	std::vector<Node*> paths;
	Node* p1=NULL, *p2=NULL;
	void clear() { paths.clear(); p1 = NULL; p2 = NULL; }
protected:
private:
};

class Supportor
{
public:

	void opitimizePointsandRibs();
public:
	Supportor();
	~Supportor();
	void generatePointsAndRibs(std::vector<std::vector<std::vector<Segment*>>>* cnts);
	void link_to_ribs(std::vector<Node*> polylines);

	std::vector<Rib*> getRibs() { return ribs; };
private:

	Path pattern;
	std::vector<Rib*> ribs;
	void add_supportting_point_for_hatchs(Paths input, int sliceid);
	void add_SP_for_rib(std::vector<Vec3f> polyin, std::vector<Vec3f>& polyout);
	typedef std::vector<Polyline> Polylines;
	std::vector<Polylines> sup_paths;
	std::vector<std::vector<Node*>> sup_nodes_;
};

