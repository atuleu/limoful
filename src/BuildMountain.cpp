#include "BuildMountain.hpp"

#include <Eigen/Dense>

typedef Eigen::MatrixXd HeightMap;


Eigen::Vector3f VertexAt(const HeightMap & hm, size_t x, size_t y) {
	float xInc = 2.0 / (hm.cols() - 1);
	float yInc = -2.0 / (hm.rows() - 1);
	return Eigen::Vector3f(x*xInc - 1.0,
	                       y*yInc + 1.0,
	                       hm(x,y));
}


Model ModelizeHeightMap( const HeightMap & hm) {
	// simply put all the points
	Model res;
	for ( size_t x = 0;
	      x < hm.cols();
	      ++x ) {
		for ( size_t y = 0;
		      y < hm.rows();
		      ++y ) {
			auto v = VertexAt(hm,x,y);

			res.Vertices.push_back(v.x());
			res.Vertices.push_back(v.y());
			res.Vertices.push_back(v.z());
		}
	}


	for ( size_t x = 0;
	      x < hm.cols() - 1 ;
	      ++x ) {
		for ( size_t y = 0;
		      y < hm.rows() - 1;
		      ++y ) {
			auto v00 = VertexAt(hm,x,y);
			auto v10 = VertexAt(hm,x+1,y);
			auto v01 = VertexAt(hm,x,y+1);
			auto v11 = VertexAt(hm,x+1,y+1);

			auto idx00 = x * hm.rows() + y;
			auto idx10 = (x+1) * hm.rows() + y;
			auto idx01 = x * hm.rows() + y + 1;
			auto idx11 = (x+1) * hm.rows() + (y+1);

			Eigen::Vector3f norm0 = (v01 - v00).cross(v10 - v00).normalized();
			Eigen::Vector3f norm1 = (v10 - v11).cross(v01 - v11).normalized();

			res.VertexIndices.push_back(idx00);
			res.VertexIndices.push_back(idx01);
			res.VertexIndices.push_back(idx10);

			size_t nIdx = res.Normals.size() / 3;
			res.NormalIndices.push_back(nIdx);
			res.NormalIndices.push_back(nIdx);
			res.NormalIndices.push_back(nIdx);

			res.Normals.push_back(norm0.x());
			res.Normals.push_back(norm0.y());
			res.Normals.push_back(norm0.z());

			res.VertexIndices.push_back(idx10);
			res.VertexIndices.push_back(idx01);
			res.VertexIndices.push_back(idx11);

			nIdx = res.Normals.size() / 3;
			res.NormalIndices.push_back(nIdx);
			res.NormalIndices.push_back(nIdx);
			res.NormalIndices.push_back(nIdx);

			res.Normals.push_back(norm1.x());
			res.Normals.push_back(norm1.y());
			res.Normals.push_back(norm1.z());
		}
	}

	return res;
}


Mountain BuildMountain(const MountainOptions & options) {
	HeightMap hm(options.GridSize,options.GridSize);

	for( size_t x = 0;
	     x < options.GridSize;
	     ++x ) {
		auto xh = float(x) / float(options.GridSize);
		for( size_t y = 0;
		     y < options.GridSize;
		     ++y ) {
			auto yh = float(y) / float(options.GridSize);
			hm(x,y) = std::max(xh,yh);
		}
	}

	Mountain res;

	res.Mountain = ModelizeHeightMap(hm);

	return res;
}
