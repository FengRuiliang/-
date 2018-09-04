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
#include "Library/clipper.hpp"
using namespace ClipperLib;
// define the kernel
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Filtered_kernel.h>
typedef CGAL::Simple_cartesian<double>    CK;
typedef CGAL::Filtered_kernel<CK>         Kernel;
// typedefs for the traits and the algorithm
#include <CGAL/Segment_Delaunay_graph_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_2.h>
typedef CGAL::Segment_Delaunay_graph_traits_2<Kernel>  Gt;
typedef CGAL::Segment_Delaunay_graph_2<Gt>             SDG2;

class Processor
{
public:
	Mesh mesh;
	std::vector<Polylines> contours;
	std::vector<std::vector<std::tuple<double,double,double,double>>> support_region_voronoi_diagrams;
public:
	Processor();
	~Processor();
	void read_obj_file(const char* file_name);
	void update_my_mesh();
	void do_slice();
	void add_support();
};

