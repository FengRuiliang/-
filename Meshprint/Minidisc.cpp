#include "Minidisc.h"
namespace MINIDISC
{
	std::pair<Vec2f, float> miniDiscWith2Point(std::vector<Vec2f> P, Vec2f q1, Vec2f q2)
	{
		std::pair<Vec2f, float> cir;
		cir.first = (q1 + q2) / 2.0f;
		cir.second = (q2 - q1).length() / 2.f;
		for (int k = 0; k < P.size(); k++)
		{
			if ((P[k] - cir.first).length() > cir.second)
			{
				float x1 = P[k].x(), x2 = q1.x(), x3 = q2.x();
				float y1 = P[k].y(), y2 = q1.y(), y3 = q2.y();
				cir.first.x() = ((y2 - y1)*(y3*y3 - y1*y1 + x3*x3 - x1*x1) - (y3 - y1)*(y2*y2 - y1*y1 + x2*x2 - x1*x1)) / (2.0*((x3 - x1)*(y2 - y1) - (x2 - x1)*(y3 - y1)));
				cir.first.y() = ((x2 - x1)*(x3*x3 - x1*x1 + y3*y3 - y1*y1) - (x3 - x1)*(x2*x2 - x1*x1 + y2*y2 - y1*y1)) / (2.0*((y3 - y1)*(x2 - x1) - (y2 - y1)*(x3 - x1)));
				cir.second = (q1 - cir.first).length();
			}
		}
		return cir;
	}

	std::pair<Vec2f, float> miniDiscWithPoint(std::vector<Vec2f> P, Vec2f q)
	{
		std::pair<Vec2f, float> cir;
		cir.first = (P.front() + q) / 2.0f;
		cir.second = (P.front() - q).length() / 2.0;
		for (int j = 1; j < P.size(); j++)
		{
			if ((P[j] - cir.first).length() > cir.second)
			{
				std::vector<Vec2f> tem;
				tem.insert(tem.end(), P.begin(), P.begin() + j - 1);
				cir = miniDiscWith2Point(tem, P[j], q);
			}
		}
		return cir;
	}

	std::pair<Vec2f, float> miniDisc(std::vector<Vec2f> pointsin)
	{
		if (pointsin.size() == 1)
		{
			return std::make_pair(pointsin.front(), 1.0);
		}
		else if (pointsin.size() == 2)
		{
			Vec2f c = 0.5f*(pointsin.front() + pointsin.back());
			float r = (pointsin.front() - pointsin.back()).length() / 2;
			return std::make_pair(c, r);
		}
		else
		{
			std::pair<Vec2f, float> cir;
			cir.first = 0.5f*(pointsin[0] + pointsin[1]);
			cir.second = (pointsin[0] - pointsin[1]).length() / 2;
			for (int i = 3; i < pointsin.size(); i++)
			{
				if ((pointsin[i] - cir.first).length() > cir.second)
				{
					std::vector<Vec2f> t;
					t.insert(t.end(), pointsin.begin(), pointsin.begin() + i - 1);
					cir = miniDiscWithPoint(t, pointsin[i]);
				}
			}
			return cir;
		}
	}
}