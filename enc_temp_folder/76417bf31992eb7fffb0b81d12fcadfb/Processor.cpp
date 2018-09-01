#include "Processor.h"


Processor::Processor()
{
}


Processor::~Processor()
{
}

void Processor::read_obj_file(const char* file_name)
{
	std::ifstream cin(file_name);
	std::string line, attribute;
	double value;
	while (getline(cin, line))
	{
		std::istringstream record(line);
		record >> attribute;
		if (attribute =="v")
		{
			std::vector<double> positon;
			while (record >> value)
				positon.push_back(value);
			if (positon.size()==3)
			{
				K::Point_3 p(positon[0], positon[1], positon[2]);
				mesh.add_vertex(p);
			}
	
		}
		else if (attribute=="f")
		{
			std::vector<Mesh::Vertex_index> vertex_id;
			while (record >> value)
			{
				vertex_id.push_back(Mesh::Vertex_index(value-1));
			}
			if (vertex_id.size() == 3)
			{
				mesh.add_face(vertex_id[0], vertex_id[1], vertex_id[2]);
			}
		}
	}
}
void Processor::update_my_mesh()
{
	Mesh::Property_map<face_descriptor, K::Vector_3> fnormals = mesh.add_property_map<face_descriptor, K::Vector_3>
		("f:normals", CGAL::NULL_VECTOR).first;
	Mesh::Property_map<vertex_descriptor, K::Vector_3> vnormals = mesh.add_property_map < vertex_descriptor, K::Vector_3 >
		("v:normals", CGAL::NULL_VECTOR).first;

	CGAL::Polygon_mesh_processing::compute_normals(mesh,
		vnormals,
		fnormals,
		CGAL::Polygon_mesh_processing::parameters::vertex_point_map(mesh.points()).
		geom_traits(K()));
}
void Processor::do_slice()
{
	if (mesh.is_empty())
	{
		return;
	}
	// Use the Slicer constructor from a pre-built AABB_tree
	AABB_tree tree(edges(mesh).first, edges(mesh).second, mesh);
	CGAL::Polygon_mesh_slicer<Mesh, K> slicer_aabb(mesh, tree);
	float z_height = 0,zmax=tree.bbox().zmax();
	while (z_height < tree.bbox().zmax())
	{
		Polylines current_polylines;
		slicer_aabb(K::Plane_3(0, 0, 1, -z_height), std::back_inserter(current_polylines));
		contours.push_back(current_polylines);
		z_height += 0.3;
	}
}
void Processor::add_support()
{
	if (contours.empty())
	{
		do_slice();
	}
}