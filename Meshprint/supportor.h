#pragma once
#include <vector>
#include "Segment.h"
#include "Sweep.h"
#include "slicer.h"
#include "clipper.hpp"
#include "Hatch.h"
#include "Minidisc.h"
using namespace ClipperLib;

class Supportor
{
public:
	Supportor();
	~Supportor();
	void generatePoint(std::vector<std::vector<std::vector<Segment*>>>* cnts);
	void add_supportting_rib_for_contours(std::vector<std::vector<std::vector<Segment*>>>* cnts);
	void add_supportting_point_for_overhangsegments(std::vector<Segment*> segments, int sliceid);

	void add_supportting_point_for_contours(std::vector<std::vector<std::vector<Segment*>>>* cnts);
	std::vector<std::vector<Vec3f>>* get_sup_points() { return sup_points; }
	std::vector<std::vector<std::vector<Vec3f>>>* get_polylines() { return polylines; };
	std::vector<std::vector<std::vector<Vec3f>>>* get_minkowssum() { return minkowskisums; };
	std::vector<std::vector<std::vector<Vec3f>>>* get_suprec() { return sup_rectangle; }
	std::vector<std::vector<std::vector<Vec3f>>>* get_suplines() { return sup_lines; }
	std::vector<std::vector<Segment*>>* getSegments() { return sup_segments; }
	std::set<HE_edge*> getEdges() { return sup_edges; }
	std::vector<std::vector<Segment>>* get_hatchs() { return hatchs; }
	
private:
	
	std::vector<std::vector<Segment>>* hatchs;
	std::vector<std::vector<Vec3f>>* sup_points;
	std::vector<std::vector<std::vector<Segment>>>* sup_ribs;
	std::vector<std::vector<std::vector<Vec3f>>>* sup_lines;
	std::vector<std::vector<Segment*>>* sup_segments;
	std::set<HE_edge*> sup_edges;
	std::vector<std::vector<std::vector<Vec3f>>>* sup_rectangle;
	std::vector<std::vector<std::vector<Vec3f>>>* polylines;
	std::vector<std::vector<std::vector<Vec3f>>>* minkowskisums;
	Path pattern;

	void merge(int num);
	void add_supportting_point_for_hatchs(Paths input, int sliceid);
	inline void findpolyline(Path target_paths, Paths mink_sum, int num);
	void add_supportting_point_for_polyline(std::vector<Vec3f> polyin, int sliceid, float err3=ERR);
	void add_SP_for_rib(std::vector<Vec3f> polyin, std::vector<Vec3f>& polyout);
	void add_supportting_point_by_uniform(Paths poly, int sliceid);
	void buildTreeStructure();
public:

private:
	Vec3f computerIntersection(Paths& mink_sum, Segment* ptr_seg);
};

