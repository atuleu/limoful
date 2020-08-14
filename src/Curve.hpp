#pragma once

#include <map>

class Curve {
public:
	static Curve Exp();

	float ValueAt(float x) const;

private:

	std::map<float,float> d_data;
};
