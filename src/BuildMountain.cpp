#include "BuildMountain.hpp"

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/create/ball_pivoting.h>
#include <vcg/complex/algorithms/create/ball_pivoting.h>

#include <Eigen/Dense>

#include <libnoise/module/perlin.h>

#include "PolarGrid.hpp"
#include "Meshifier.hpp"

float rad2deg(float a) { return a * 180.0 / M_PI; }

class FittedCurves {
public:
	FittedCurves(const MountainOptions & options)
		: d_minSlope(M_PI / 180.0 * options.SlopeMinAngle)
		, d_maxSlope(M_PI / 180.0 * options.SlopeMaxAngle)
		, d_jump(options.EdgeJump)
		, d_minValue(options.LowMin) {
		if ( d_maxSlope < d_minSlope) {
			std::swap(d_minSlope,d_maxSlope);
		}

		float angleIncrement = PolarGrid::AngleIncrement(options.GridSize);

		size_t i = 0;
		for ( const auto & angle : options.Angles ) {
			float angleExact = std::floor(angle * M_PI / ( angleIncrement * 180.0) ) * angleIncrement;
			d_curves[angleExact] = options.Curves[i];
			++i;
		}

		auto firstAngle = d_curves.begin()->first;
		auto firstCurve = d_curves.begin()->second;
		auto lastAngle = (--d_curves.end())->first;
		auto lastCurve = (--d_curves.end())->second;
		d_curves[firstAngle + 2*M_PI] = firstCurve;
		d_curves[lastAngle - 2*M_PI] = lastCurve;
	}


	std::pair<float,float> MinAndMaxHeight(float length,float angle) {
		const auto & [ lowCurve,highCurve,lowAngle,highAngle] = FindBoundingCurves(angle);



		auto mmin = mix(interpolateMin(lowCurve,lowAngle,length,angle),
		                interpolateMin(highCurve,highAngle,length,angle));

		auto mmax = mix(interpolateMax(lowCurve,lowAngle,length,angle),
		                interpolateMax(highCurve,highAngle,length,angle));

		return {mmin,mmax};
	}

private:
	std::tuple<Curve*,Curve*,float,float> FindBoundingCurves(float angle) {
		while ( angle < 0.0 ) {
			angle += 2*M_PI;
		}
		while ( angle >= 2 * M_PI ) {
			angle -= 2*M_PI;
		}
		auto fi = d_curves.upper_bound(angle);
		if ( fi == d_curves.begin() || fi == d_curves.end() ) {
			throw std::logic_error("Oupsie for " + std::to_string(angle));
		}
		auto ffi = fi;
		--ffi;
		return {&(ffi->second),&(fi->second),ffi->first,fi->first};
	}

	std::pair<float,float> interpolateMax(Curve * curve, float angleCurve, float length, float angle) {
		float distAngle = std::fabs(angle - angleCurve);
		if ( distAngle > 2*M_PI ) { distAngle -= 2*M_PI; }
		float distToCurve = distAngle * length;
		float v = curve->ValueAt(1.0-length);

		return {distToCurve,v - std::tan(d_minSlope) * distToCurve};
	}

	std::pair<float,float> interpolateMin(Curve * curve, float angleCurve, float length, float angle) {
		float distAngle = std::fabs(angle - angleCurve);
		if ( distAngle > 2*M_PI ) { distAngle -= 2*M_PI; }
		float distToCurve = distAngle * length;
		float v = curve->ValueAt(1.0-length);
		if ( distAngle < 1e-6 ) {
			return {distToCurve,v};
		}
		return {distToCurve,std::max(v + d_jump - std::tan(d_maxSlope) * distToCurve,d_minValue)};
	}

	float mix(std::pair<float,float> a,
	          std::pair<float,float> b) {

		float sum = a.first + b.first;
		auto res =  (a.second * b.first + b.second * a.first) / sum;
		return res;
	}


	std::map<float,Curve> d_curves;
	float d_minSlope,d_maxSlope,d_jump,d_minValue;
};


std::vector<QImage> DrawNoise(size_t gridSize,
                              size_t nbOctaves,
                              size_t seed) {
	std::vector<QImage> res;
	noise::module::Perlin n;
	float octave = 1.0;
	for ( size_t i = 0; i < nbOctaves; ++i) {
		QImage image(gridSize,gridSize,QImage::Format_Grayscale8);
		for ( size_t x = 0; x < gridSize; ++x) {
			for ( size_t y = 0; y < gridSize; ++y) {
				auto v = (n.GetValue(float(x)/float(gridSize-1)*octave,
				                    float(y)/float(gridSize-1)*octave,
				                     seed) + 1) / 2.0;
				v = std::min(std::max(v,0.0),1.0);
				image.setPixelColor(x,y,QColor(255*v,255*v,255*v));
			}
		}
		octave *= 2.0;
		res.push_back(image);
	}
	return res;
}

std::pair<size_t,size_t> PolarToImage(float length,float angle,size_t gridSize) {
	float x = (length * std::cos(angle) + 1.0) /2.0;
	float y = (length * std::sin(angle) + 1.0) /2.0;
	return {std::min(size_t(x*gridSize),gridSize-1),
	        std::min(size_t(y*gridSize),gridSize-1)};
}

std::pair<PolarGrid::Grid,
          std::vector<size_t> > RemovePointsBelow(const PolarGrid::Grid & points,
                                                  PolarGrid::RayMap rays,
                                                  float value) {
	// cut each ray at value
	for ( auto & [angle,indexes] : rays ) {
		size_t newSize = 0;
		for ( const auto & idx : indexes ) {
			const auto & p = points[idx];
			if ( p.z() < value ) {
				indexes.resize(newSize);
				break;
			} else {
				++newSize;
			}
		}
	}
	PolarGrid::Grid res = {points[0]};
	std::vector<size_t> boundary;
	for ( const auto & [angle,indexes] : rays ) {
		for ( const auto & idx : indexes ) {
			res.push_back(points[idx]);
		}
		if ( indexes.empty() == false ) {
			boundary.push_back(indexes.back());
		}
	}
	return {res,boundary};
}

Mountain BuildMountain(MountainOptions options) {
	if ( options.Curves.size() == 0 ) {
		options.Curves = {Curve::Exp(),Curve::Exp()};
		options.Angles = {0,180.0};
	}

	if ( options.Curves.size() == 1 ) {
		options.Curves.push_back(Curve::Exp());
		options.Angles.push_back(180.0);
	}

	auto curves = FittedCurves(options);

	double totalWeight = 0.0;
	for ( const auto & w : options.OctaveWeights ) {
		totalWeight += w;
	}
	for ( auto & w : options.OctaveWeights ) {
		w /= totalWeight;
	}

	Mountain res;
	res.Noises = DrawNoise(options.GridSize,
	                       options.OctaveWeights.size(),
	                       options.Seed);
	auto [points,rays] = PolarGrid::Build(options.GridSize);
	res.Points = points;
	std::vector<Eigen::Vector3f> maximumHeight,minimumHeight;
	float maxZ(-3000),minZ(3000);
	for ( auto & p : res.Points ) {
		auto [minHeight,maxHeight] = curves.MinAndMaxHeight(p.x(),p.y());

		maximumHeight.push_back(Eigen::Vector3f(p.x(),p.y(),maxHeight));
		minimumHeight.push_back(Eigen::Vector3f(p.x(),p.y(),minHeight));

		//compute noise
		auto [x,y] = PolarToImage(p.x(),p.y(),options.GridSize);
		float noise = 0.0;
		for ( size_t i = 0; i < options.OctaveWeights.size(); ++i ) {
			noise += options.OctaveWeights[i] * float(qGray(res.Noises[i].pixel(x,y)))/255.0;
		}

		p.z() = minHeight + (maxHeight - minHeight) * noise;
		minZ = std::min(noise,minZ);
		maxZ = std::max(noise,maxZ);

	}
	std::cerr << "min Noise: " << minZ << " max Noise: " << maxZ << std::endl;
	auto [newPoints,boundaries] = RemovePointsBelow(res.Points,rays,options.BaseCut);
	res.Points = newPoints;
	PolarGrid::ToCartesian(res.Points);
	PolarGrid::ToCartesian(maximumHeight);
	PolarGrid::ToCartesian(minimumHeight);

	Meshifier mountainMeshifier(res.Points,boundaries);
	Meshifier enveloppeMeshifier(maximumHeight);


	res.Mountain = mountainMeshifier.BuildCompleteMesh(res.Points);

	res.Maximum =  enveloppeMeshifier.BuildTopMesh(maximumHeight);
	res.Minimum =  enveloppeMeshifier.BuildTopMesh(minimumHeight);
	return res;
}
