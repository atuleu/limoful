#pragma once

#include <QImage>
#include "Viewer.hpp"
#include <vector>
#include <Eigen/Dense>
#include "Curve.hpp"

struct Mountain {
	std::vector<QImage> Noises;
	std::vector<Eigen::Vector3f> Points;
	LIFMesh::Ptr Maximum,Minimum,Mountain;
};


struct MountainOptions {
	std::vector<Curve> Curves;
	std::vector<float> Angles;

	float SlopeMinAngle,SlopeMaxAngle;

	size_t GridSize;
	size_t Seed;
	std::vector<float> OctaveWeights;
	double BallPivotingRadius,BallPivotingCluster,BallPivotingAngle;
};



Mountain BuildMountain(MountainOptions options);
