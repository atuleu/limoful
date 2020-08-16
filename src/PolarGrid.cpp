#include "PolarGrid.hpp"


std::pair<PolarGrid::Grid,
          PolarGrid::RayMap> PolarGrid::Build(size_t gridSize) {

	gridSize = ActualGridSize(gridSize);
	if ( gridSize < 2 ) {
		return {{},{}};
	}

	RayMap rays;


	float angleIncrement = AngleIncrement(gridSize);
	float lengthIncrement = 2.0 / (gridSize - 1);

	std::vector<Eigen::Vector3f> grid = { Eigen::Vector3f(0,0,0) };
	std::vector<float> lastAngles((gridSize-1),0.0);
	for ( size_t angleIdx = 0; angleIdx < gridSize; ++angleIdx ) {
		float angle = angleIdx * angleIncrement;
		auto c = std::fabs(std::cos(angle));
		auto s = std::fabs(std::sin(angle));
		for ( size_t lengthIdx = 1; lengthIdx <= (gridSize - 1) ; ++lengthIdx) {
			float length = lengthIdx * lengthIncrement;
			if (c*length > 1.0 || s*length > 1.0 ) {
				continue;
			}
			if ( angleIdx == 0 ) {
				rays[angle].push_back(grid.size());
				grid.push_back(Eigen::Vector3f(length,angle,0.0));
				continue;
			}
			if ( (angle - lastAngles[lengthIdx-1]) * length < lengthIncrement ) {
				continue;
			}
			lastAngles[lengthIdx-1] = angle;
			rays[angle].push_back(grid.size());
			grid.push_back(Eigen::Vector3f(length,angle,0.0));
		}
	}

	return {grid,rays};
}


void PolarGrid::ToCartesian(std::vector<Eigen::Vector3f> & points ) {
	for ( auto & p : points ) {
		p = Eigen::Vector3f(p.x() * std::cos(p.y()),
		                    p.x() * std::sin(p.y()),
		                    p.z());
	}
}
