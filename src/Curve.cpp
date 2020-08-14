#include "Curve.hpp"

#include <limits>
#include <cmath>


Curve Curve::Exp() {
	Curve res;

	for ( size_t i = 0; i <= 30; ++i ) {
		float x = float(i) / 30.0;
		res.d_data[1.0 - x] = std::exp(-4*x);
	}
	return res;
}


float Curve::ValueAt(float x) const {

	auto bound = d_data.upper_bound(x);
	if ( bound == d_data.cend() ) {
		if ( d_data.empty() ) {
			return 0;
		}
		return (--bound)->second;
	}

	if ( bound == d_data.begin() ) {
		return bound->second;
	}
	float highX = bound->first;
	float highY = bound->second;
	--bound;
	float lowX = bound->first;
	float lowY = bound->second;
	return (x-lowX) / (highX-lowX) * (highY - lowY) + lowY;
}
