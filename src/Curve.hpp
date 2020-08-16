#pragma once

#include <map>

class Curve {
public:
	static Curve Exp();

	static std::map<std::string,Curve> AllCurves();


	float ValueAt(float x) const;


private:

	std::map<float,float> d_data;
};
