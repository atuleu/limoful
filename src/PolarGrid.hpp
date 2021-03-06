#pragma once

#include <Eigen/Core>


class PolarGrid {
public:
	static std::vector<Eigen::Vector3f> Build(size_t gridSize);

	static void ToCartesian(std::vector<Eigen::Vector3f> & points);

	static size_t ActualGridSize(size_t gridSize) {
		if ( gridSize % 2 == 0 ) {
			return gridSize + 1;
		}
		return gridSize;
	}

	static float AngleIncrement(size_t gridSize) {
		return 2.0 * M_PI / ActualGridSize(gridSize);
	}


};
