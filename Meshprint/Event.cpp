#include "Event.h"



Event::Event()
{
}

Event::Event(vec3 pos_)
{
	_pos = vec2(pos_.x(),pos_.y());
}


Event::~Event()
{
}
