#include "BuildMountain.hpp"

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/create/ball_pivoting.h>
#include <vcg/complex/algorithms/create/ball_pivoting.h>

#include <Eigen/Dense>

#include <libnoise/module/perlin.h>

#include "PolarGrid.hpp"
#include "Meshifier.hpp"


template <typename T> T smooth_step(T x, T e0, T e1) {
	x = std::clamp((x - e0) / (e1 - e0),T(0.0),T(1.0));
	return x * x * (3.0-2.0*x);
}

template <typename T> T smooth_min(T a, T b, T radius) {
	T r = (b-a) / radius;
	r = r * r;
	T cor = 0.0;
	if ( r < 1.0  ) {
		cor = 0.0;
		cor =  radius / 8.0 * std::exp(r/(r-1.0));
	}
	return smooth_step(a,b - radius, b + radius) * (b - a) + a - cor;
}

float rad2deg(float a) { return a * 180.0 / M_PI; }

class FittedCurves {
public:
	FittedCurves(const MountainOptions & options)
		: d_minSlopeTop(M_PI / 180.0 * options.SlopeMinTop)
		, d_minSlopeBot(M_PI / 180.0 * options.SlopeMinBot)
		, d_maxSlopeTop(M_PI / 180.0 * options.SlopeMaxTop)
		, d_maxSlopeBot(M_PI / 180.0 * options.SlopeMaxBot)
		, d_jump(options.EdgeJump)
		, d_minValue(options.LowMin)
		, d_radius(options.SmoothRadius) {

		float angleIncrement = PolarGrid::AngleIncrement(options.GridSize);

		size_t i = 0;
		for ( const auto & angle : options.Angles ) {
			float angleExact = std::floor(angle * M_PI / ( angleIncrement * 180.0) ) * angleIncrement;
			d_curves.insert({angleExact,options.Curves[i]});
			++i;
		}

		auto firstAngle = d_curves.begin()->first;
		auto firstCurve = d_curves.begin()->second;
		auto lastAngle = (--d_curves.end())->first;
		auto lastCurve = (--d_curves.end())->second;
		d_curves.insert({firstAngle + 2*M_PI,firstCurve});
		d_curves.insert({lastAngle - 2*M_PI,lastCurve});
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
		Eigen::Vector3d p(length * std::cos(angle),length * std::sin(angle),0),
			r(std::cos(angleCurve),std::sin(angleCurve),0);
		float distToCurve = std::abs(angle - angleCurve);
		while ( distToCurve >= 2*M_PI ) { distToCurve -= 2*M_PI; }
		while ( distToCurve < 0 ) { distToCurve += 2*M_PI; }
		float d = std::abs(p.dot(r));
		float v = curve->ValueAt(1.0-d);
		float ds = p.cross(r).norm();
		d /= length;
		return {distToCurve,smooth_min(v-std::tan(d_maxSlopeBot) * ds,curve->ValueAt(1.0)-std::tan(d_maxSlopeTop) * ds, d_radius) };
	}

	std::pair<float,float> interpolateMin(Curve * curve, float angleCurve, float length, float angle) {
		Eigen::Vector3d p(length * std::cos(angle),length * std::sin(angle),0),
			r(std::cos(angleCurve),std::sin(angleCurve),0);
		float distToCurve = std::abs(angle - angleCurve);
		while ( distToCurve >= 2*M_PI ) { distToCurve -= 2*M_PI; }
		while ( distToCurve < 0 ) { distToCurve += 2*M_PI; }
		float d = std::abs(p.dot(r));
		float v = curve->FilteredValueAt(1.0-d);
		float ds = p.cross(r).norm();
		d /= length;

		float vv = smooth_min(v-std::tan(d_minSlopeBot) * ds,curve->ValueAt(1.0)-std::tan(d_minSlopeTop) * ds,d_radius);

		return {distToCurve,std::max(vv + d_jump,d_minValue)};
	}

	float mix(std::pair<float,float> a,
	          std::pair<float,float> b) {
		float r = a.first / (a.first + b.first);
		if ( r < 0.2 ) { return a.second; }
		if ( r > 0.8 ) { return b.second; }
		r -= 0.2;
		r /= 0.6;
		return r * (b.second - a.second) + a.second;
	}


	std::map<float,Curve> d_curves;
	float d_minSlopeTop,d_minSlopeBot,d_maxSlopeTop,d_maxSlopeBot,d_jump,d_minValue,d_radius;
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

void RemovePointsBelow(std::vector<Eigen::Vector3f> & points,float value) {
	points.erase(std::remove_if(points.begin(),
	                            points.end(),
	                            [value](const Eigen::Vector3f & p) {
		                            return p.z() < value;
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

	RemovePointsBelow(res.Points,options.BaseCut);
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
