#pragma once
enum SupporttingPointType
{
	UNIFORM,
	ALONGPATH
};

class Settings
{
public:
	Settings();
	~Settings();
	
	static SupporttingPointType point_type;

};

