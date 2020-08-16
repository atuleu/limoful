#include "BuildMountain.hpp"

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/create/ball_pivoting.h>
#include <vcg/complex/algorithms/create/ball_pivoting.h>

#include <Eigen/Dense>

#include <libnoise/module/perlin.h>

#include "PolarGrid.hpp"
#include "Meshifier.hpp"

class FittedCurves {
public:
	FittedCurves(const MountainOptions & options)
		: d_minSlope(M_PI / 180.0 * options.SlopeMinAngle)
		, d_maxSlope(M_PI / 180.0 * options.SlopeMaxAngle) {
		if ( d_maxSlope < d_minSlope) {
			std::swap(d_minSlope,d_maxSlope);
		}

		float angleIncrement = PolarGrid::AngleIncrement(options.GridSize);

		size_t i = 0;
		for ( const auto & angle : options.Angles ) {
			float angleExact = std::round(angle * M_PI / ( angleIncrement * 180.0) ) * angleIncrement;
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
		return {std::max(interpolateMin(lowCurve,lowAngle,length,angle),
		                 interpolateMin(highCurve,highAngle,length,angle)),
		        std::max(interpolateMax(lowCurve,lowAngle,length,angle),
		                 interpolateMax(highCurve,highAngle,length,angle))};
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

	float interpolateMax(Curve * curve, float angleCurve, float length, float angle) {
		float distToCurve = std::fabs(angle - angleCurve) * length;
		return curve->ValueAt(1.0-length) - std::tan(d_minSlope) * distToCurve;
	}

	float interpolateMin(Curve * curve, float angleCurve, float length, float angle) {
		float distToCurve = std::fabs(angle - angleCurve) * length;
		float v = curve->ValueAt(1.0-length);
		return std::max(v  - std::tan(d_maxSlope) * distToCurve,float(-0.1));
	}


	std::map<float,Curve> d_curves;
	float d_minSlope,d_maxSlope;
};


std::vector<QImage> DrawNoise(size_t gridSize,
                              size_t nbOctaves,
                              size_t seed) {
	std::vector<QImage> res;
	noise::module::Perlin n;
	float octave = 2.0;
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

void RemovePointsBelow0(std::vector<Eigen::Vector3f> & points) {
	points.erase(std::remove_if(points.begin(),
	                            points.end(),
	                            [](const Eigen::Vector3f & p) {
		                            return p.z() < 0;
	                            }),
	             points.end());
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
	res.Points = PolarGrid::Build(options.GridSize);
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

	RemovePointsBelow0(res.Points);
	PolarGrid::ToCartesian(res.Points);
	PolarGrid::ToCartesian(maximumHeight);
	PolarGrid::ToCartesian(minimumHeight);

	Meshifier mountainMeshifier(res.Points);
	Meshifier enveloppeMeshifier(maximumHeight);


	res.Mountain = mountainMeshifier.BuildCompleteMesh(res.Points);

	res.Maximum =  enveloppeMeshifier.BuildTopMesh(maximumHeight);
	res.Minimum =  enveloppeMeshifier.BuildTopMesh(minimumHeight);
	return res;
}
