#pragma once
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <iostream>
#include <fstream>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor   face_descriptor;
#include <CGAL/AABB_halfedge_graph_segment_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <fstream>
typedef std::vector<K::Point_3> Polyline_type;
typedef std::list< Polyline_type > Polylines;
typedef CGAL::AABB_halfedge_graph_segment_primitive<Mesh> HGSP;
typedef CGAL::AABB_traits<K, HGSP>    AABB_traits;
typedef CGAL::AABB_tree<AABB_traits>  AABB_tree;
#include<boost/shared_ptr.hpp>
#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_with_holes_2.h>
#include<CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/arrange_offset_polygons_2.h>
typedef CGAL::Polygon_2<K>            Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes;
typedef CGAL::Straight_skeleton_2<K>  Ss;
typedef boost::shared_ptr<Polygon_2> PolygonPtr;
typedef boost::shared_ptr<Ss> SsPtr;
typedef std::vector<PolygonPtr> PolygonPtrVector;


class Processor
{
public:
	Mesh mesh;
	std::vector<Polylines> contours;
public:
	Processor();
	~Processor();
	void read_obj_file(const char* file_name);
	void update_my_mesh();
	void do_slice();
	void add_support();
};

