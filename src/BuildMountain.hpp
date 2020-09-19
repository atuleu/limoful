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

	float SlopeMinTop,SlopeMinBot,SlopeMaxTop,SlopeMaxBot,EdgeJump,BaseCut,LowMin,SmoothRadius;

	size_t GridSize;
	size_t Seed;
	std::vector<float> OctaveWeights;
};



Mountain BuildMountain(MountainOptions options);
