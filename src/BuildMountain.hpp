#pragma once

#include <QImage>
#include "Viewer.hpp"
#include <vector>
#include "Curve.hpp"

struct Mountain {
	std::vector<QImage> Noises;
	Model Maximum;
	Model Minimum;
	Model Mountain;
};


struct MountainOptions {
	std::vector<Curve> Curves;
	std::vector<float> Angles;

	float SlopeMinAngle,SlopeMaxAngle;

	size_t GridSize;
	size_t Seed;
	std::vector<float> OctaveWeights;
};



Mountain BuildMountain(const MountainOptions & options);
