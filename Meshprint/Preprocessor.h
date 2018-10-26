#pragma once
#include "slicer.h"
#include "Supportor.h"
#include "HE_mesh/Mesh3D.h"
class Preprocessor
{
private:
	Mesh3D* tar;
	std::vector<std::vector<std::vector<Segment*>>> cnt;
public:
	Preprocessor();
	Preprocessor(Mesh3D* param1) { tar = param1; }
	~Preprocessor();
	void do_slice();
	void add_support();
};

