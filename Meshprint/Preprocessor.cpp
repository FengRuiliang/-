#include "Preprocessor.h"



Preprocessor::Preprocessor()
{
}


Preprocessor::~Preprocessor()
{
}

void Preprocessor::do_slice()
{
	Slicer sl(tar);

	sl.execute();

	cnt = sl.get_contours();
}

void Preprocessor::add_support()
{
	if (cnt->empty())
	{
		do_slice();
	}
	Supportor su;
	su.add_support_point(*cnt);
}
