#include "Curve.hpp"

#include <limits>
#include <cmath>

#include <QFile>
#include <QTextStream>

Curve Curve::Exp() {
	Curve res;

	for ( size_t i = 0; i <= 30; ++i ) {
		float x = float(i) / 30.0;
		res.d_data[1.0-x] = std::exp(-2*x);
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

std::map<std::string,Curve> Curve::AllCurves() {
	QFile curvesFile(":curves.csv");

	if ( curvesFile.open(QIODevice::ReadOnly) == false ) {
		return {};
	}

	std::vector<std::pair<std::string,std::vector<float>>> curvePoints;

	QTextStream in(&curvesFile);
	while(in.atEnd() == false ) {
		auto valuesString = in.readLine().split(",");
		if ( valuesString.empty() ) {
			continue;
		}
		if (curvePoints.empty() == true ) {
			for ( size_t i = 1; i < valuesString.size(); ++i ) {
				curvePoints.push_back({valuesString[i].toUtf8().constData(),{}});
			}
			continue;
		}
		float x = valuesString[0].toFloat() / 1.5;
		if ( x < 0.0 || x > 1.0 ) {
			continue;
		}
		for ( size_t i = 1; i < valuesString.size() ; ++i ) {
			if ( valuesString[i].isEmpty() == true ) { continue; }
			float y = valuesString[i].toFloat() / 1.5;
			y = std::min(std::max(y,float(0.0)),float(1.0));
			curvePoints[i-1].second.push_back(x);
			curvePoints[i-1].second.push_back(y);
		}
	}

	std::map<std::string,Curve> res;
	for ( const auto & [name,points] : curvePoints ) {
		Curve c;
		for ( size_t i = 0; i < points.size() / 2; ++i ) {
			c.d_data[points[2*i]] = points[2*i+1];
		}
		res[name] = c;
	}
	return res;
}
