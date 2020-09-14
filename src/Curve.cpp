#include "Curve.hpp"

#include <limits>
#include <cmath>

#include <QFile>
#include <QTextStream>

#include <iostream>


Curve Curve::Exp() {
	std::vector<std::pair<float,float>> xy;


	for ( size_t i = 0; i <= 30; ++i ) {
		float x = float(i) / 30.0;
		xy.push_back({1.0-x,-2*x});
	}

	return Curve(xy,xy);
}


float Curve::ValueAt(const std::map<float,float> & data,float x) {

	auto bound = data.upper_bound(x);
	if ( bound == data.cend() ) {
		if ( data.empty() ) {
			return 0;
		}
		return (--bound)->second;
	}

	if ( bound == data.begin() ) {
		return bound->second;
	}

	float highX = bound->first;
	float highY = bound->second;
	--bound;
	float lowX = bound->first;
	float lowY = bound->second;

	return (x-lowX) / (highX-lowX) * (highY - lowY) + lowY;
}


float Curve::ValueAt(float x) const {
	return ValueAt(d_data,x);
}


float Curve::FilteredValueAt(float x) const {
	return ValueAt(d_filtered,x);
}


std::map<std::string,Curve> Curve::AllCurves() {
	QFile curvesFile(":curves_smoothed.csv");

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

	std::map<std::string,std::vector<float>> pointsByNames(curvePoints.begin(),curvePoints.end());

	std::map<std::string,Curve> res;

	std::vector<std::pair<float,float>> xy,xySmoothed;

	for ( const auto & [name,points] : pointsByNames ) {
		if ( QString(name.c_str()).endsWith("-smoothed") == true ) {
			continue;
		}
		xy.clear();
		xySmoothed.clear();
		xy.reserve(points.size()/2);
		xySmoothed.reserve(points.size()/2);
		const auto & pointsSmoothed = pointsByNames[name+"-smoothed"];
		for ( size_t i = 0; i < points.size() / 2; ++i ) {
			xy.push_back({points[2*i],points[2*i+1]});
			xySmoothed.push_back({pointsSmoothed[2*i], pointsSmoothed[2*i+1]});
		}
		res.insert({name,Curve(xy,xySmoothed)});
	}

	return res;
}


Curve::Curve(const std::vector<std::pair<float,float>> & xy,
             const std::vector<std::pair<float,float>> & xySmoothed)
	: d_data(xy.begin(),xy.end())
	, d_filtered(xySmoothed.begin(),xySmoothed.end()) {
}
