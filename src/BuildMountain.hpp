#pragma once

#include <QImage>
#include "Viewer.hpp"
#include <vector>
#include "Curve.hpp"

struct Mountain {
	std::vector<QImage> Noises;
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



Mountain BuildMountain(const MountainOptions & options);
