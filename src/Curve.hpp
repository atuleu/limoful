#pragma once

#include <map>
#include <vector>

class Curve {
public:
	static Curve Exp();

	static std::map<std::string,Curve> AllCurves();

	Curve(const std::vector<std::pair<float,float>> & xy,
	      const std::vector<std::pair<float,float>> & smoothed);

	Curve() {};

	float ValueAt(float x) const;
	float FilteredValueAt(float x) const;

private:
	static float ValueAt(const std::map<float,float> & data,float x);


	std::map<float,float> d_data;
	std::map<float,float> d_filtered;
};
