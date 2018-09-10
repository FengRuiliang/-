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

// includes for defining the Voronoi diagram adaptor
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Segment_Delaunay_graph_Linf_filtered_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_Linf_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_policies_2.h>
// typedefs for defining the adaptor
typedef CGAL::Segment_Delaunay_graph_Linf_filtered_traits_2<K>               Gt;
typedef CGAL::Segment_Delaunay_graph_Linf_2<Gt>                              DT;
typedef CGAL::Segment_Delaunay_graph_adaptation_traits_2<DT>                 AT;
typedef CGAL::Segment_Delaunay_graph_degeneracy_removal_policy_2<DT>         AP;
typedef CGAL::Voronoi_diagram_2<DT, AT, AP>                                    VD;
// typedef for the result type of the point location
typedef AT::Site_2                    Site_2;
typedef AT::Point_2                   Point_2;
typedef VD::Locate_result             Locate_result;
typedef VD::Vertex_handle             Vertex_handle;
typedef VD::Face_handle               Face_handle;
typedef VD::Halfedge_handle           Halfedge_handle;
typedef VD::Ccb_halfedge_circulator   Ccb_halfedge_circulator;


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

