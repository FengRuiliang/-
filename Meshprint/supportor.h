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
	std::vector<Rib *> ribs;
	HE_edge* edge_;
	Vec3f normal_;
	Segment* seg_;
	Node* next_=NULL;

public:
	Node(Vec3f in_) { 
		pos = in_;
	};
	~Node() {};
	Node(Segment* in_)
	{
		pos = in_->get_v2();
		seg_ = in_;
		edge_ = in_->get_edge();
		normal_ = edge_->pvert_->position()-edge_->start_->position();
		normal_.normalize();
	};
	HE_edge* Edge() { return edge_; }
	Vec3f&    Normal() { return normal_; }
	Node* setNext(Node* in_) { return next_ = in_; }
	Node* getNext() { return next_; }
	std::vector<Rib*>&	getRibs() { return ribs; }

	Segment* getSegments() { return seg_; }
	Vec3f getPosition() { return pos; }
	void setRib(Rib* rib_in_)
	{
		ribs.push_back(rib_in_);
	};
	
	bool isHeadforALlRibs();

	void beRemovedFromRibs();
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
		nodes.push_back(ni_);
	};
	~Rib() {};
	HE_edge* Edge() { return edge_; }
	Vec3f& Normal() { return normal_; }
	Node* CandidateNode() { return candidate_node; }
	std::vector<Node*>& Nodes() { return nodes; }
	void popFront() { nodes.erase(nodes.begin()); }
	float Angle() { return angle_; }
	void setAngle(float in) { angle_ = in; }
	void setCandidateNode(Node* node_in_) { candidate_node = node_in_; }
	void setEdge(HE_edge* in_) { edge_ = in_; };
	Node* getHeadNodePtr() { return nodes.empty()? NULL:nodes.front(); }
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

	void opitimizePointsandRibs();
public:
	Supportor();
	~Supportor();
	void generatePointsAndRibs(std::vector<std::vector<std::vector<Segment*>>>* cnts);
	void link_to_ribs(std::vector<Node*> polylines);

	std::vector<Rib*> getRibs() { return ribs; };
	std::vector<std::vector<Node*>> getNodes() { return sup_nodes_; }
	std::vector< std::vector<std::vector<Node*>>>  getPaths() { return sup_paths; }
private:

	Path pattern;
	std::vector<Rib*> ribs;
	std::vector< std::vector<std::vector<Node*>>> sup_paths;
	std::vector<std::vector<Node*>> sup_nodes_;
	std::vector<std::vector<Node*>> candidate_nodes_;
};

