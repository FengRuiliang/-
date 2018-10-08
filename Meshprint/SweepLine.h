#pragma once
#include "queue"
template<typename T>
class SweepLine
{
private:
	struct Event
	{
		std::vector<T*> U;

	};
public:
	SweepLine();
	~SweepLine();

private:
	std::queue<Event> Q;
};

