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
public:
	Node(Vec3f pos_in_, HE_edge* edge_in)
	{
		pos = pos_in_;
		rib = NULL;
		edge_ = edge_in;
		normal_ = edge_in->pvert_->position() - edge_in->start_->position();
		normal_.normalize();
	};
	~Node() {};
	HE_edge* Edge() { return edge_; }
	Vec3f&    Normal() { return normal_; }
	Rib*	getRib() { return rib; }
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

		normal_ = ni_->Normal();
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


class Supportor
{
public:

public:
	Supportor();
	~Supportor();
	void generatePointsAndRibs(std::vector<std::vector<std::vector<Segment*>>>* cnts);
	void link_to_ribs(std::vector<Node*> nodes_);
	void add_supportting_rib_for_contours(std::vector<std::vector<std::vector<Segment*>>>* cnts);
	void add_supportting_point_for_overhangsegments(std::vector<Segment*> segments, int sliceid);
	
	void add_supportting_point_for_contours(std::vector<std::vector<std::vector<Segment*>>>* cnts);
	std::vector<std::vector<Vec3f>>* get_sup_points() { return sup_points; }
	std::vector<std::vector<std::vector<Vec3f>>>* get_polylines() { return polylines; };
	std::vector<std::vector<std::vector<Vec3f>>>* get_minkowssum() { return minkowskisums; };
	std::vector<std::vector<std::vector<Vec3f>>>* get_suprec() { return sup_rectangle; }
	std::vector<std::vector<std::vector<Vec3f>>>* get_suplines() { return sup_lines; }
	std::vector<std::vector<std::vector<Segment>>>* getSupPaths() { return sup_paths; }
	std::vector<std::vector<Segment*>>* getSegments() { return sup_segments; }
	std::set<HE_edge*> getEdges() { return sup_edges; }
	std::vector<std::vector<Segment>>* get_hatchs() { return hatchs; }
	std::vector<Rib*> getRibs() { return ribs; };
private:
	
	std::vector<std::vector<Segment>>* hatchs;
	std::vector<std::vector<Vec3f>>* sup_points;
	std::vector<std::vector<std::vector<Segment>>>* sup_paths;
	std::vector<std::vector<std::vector<Vec3f>>>* sup_lines;
	std::vector<std::vector<Segment*>>* sup_segments;
	std::set<HE_edge*> sup_edges;
	std::vector<std::vector<std::vector<Vec3f>>>* sup_rectangle;
	std::vector<std::vector<std::vector<Vec3f>>>* polylines;
	std::vector<std::vector<std::vector<Vec3f>>>* minkowskisums;
	Path pattern;
	std::vector<Rib*> ribs;
	void merge(int num);
	void add_supportting_point_for_hatchs(Paths input, int sliceid);
	inline void findpolyline(Path target_paths, Paths mink_sum, int num);
	void add_supportting_point_for_polyline(std::vector<Vec3f> polyin, int sliceid, float err3=ERR);
	void add_SP_for_rib(std::vector<Vec3f> polyin, std::vector<Vec3f>& polyout);
	void add_supportting_point_by_uniform(Paths poly, int sliceid);
	void buildTreeStructure();

	void updateRibs();
private:
	Vec3f computerIntersection(Paths& mink_sum, Segment* ptr_seg);

};

