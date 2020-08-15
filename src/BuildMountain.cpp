#include "BuildMountain.hpp"

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/create/ball_pivoting.h>

#include <Eigen/Dense>


typedef Eigen::MatrixXd HeightMap;


Eigen::Vector3f VertexAt(const HeightMap & hm, size_t x, size_t y) {
	float xInc = 2.0 / (hm.cols() - 1);
	float yInc = -2.0 / (hm.rows() - 1);
	return Eigen::Vector3f(x*xInc - 1.0,
	                       y*yInc + 1.0,
	                       hm(x,y));
}


LIFMesh::Ptr ModelizeHeightMap( const HeightMap & hm, double radius, double clusterRatio, double angle) {
	auto mesh = std::make_shared<LIFMesh>();
	// simply put all the points
	for ( size_t x = 0;
	      x < hm.cols();
	      ++x ) {
		for ( size_t y = 0;
		      y < hm.rows();
		      ++y ) {
			auto v = VertexAt(hm,x,y);
			if ( v.z() < 0.0 ) {
				continue;
			}
			vcg::tri::Allocator<LIFMesh>::AddVertex(*mesh,LIFMesh::CoordType(v.x(),v.y(),v.z()));
			vcg::tri::Allocator<LIFMesh>::AddVertex(*mesh,LIFMesh::CoordType(v.x(),v.y(),0.0));
		}
	}

	vcg::tri::BallPivoting<LIFMesh> pivot(*mesh,radius,clusterRatio,angle * M_PI/ 180.0);
	pivot.BuildMesh();
	vcg::tri::UpdateNormal<LIFMesh>::PerFaceNormalized(*mesh);
	for ( auto & f : mesh->face ) {
		//f.N() *= -1.0;
	}
	return mesh;
}


Mountain BuildMountain(const MountainOptions & options) {
	HeightMap hm(options.GridSize,options.GridSize);

	for( size_t x = 0;
	     x < options.GridSize;
	     ++x ) {
		auto xh = float(x) / float(options.GridSize) - 0.5;
		for( size_t y = 0;
		     y < options.GridSize;
		     ++y ) {
			auto yh = float(y) / float(options.GridSize) - 0.5;
			hm(x,y) = std::max(xh,yh);
		}
	}

	Mountain res;

	res.Mountain = ModelizeHeightMap(hm,
	                                 options.BallPivotingRadius,
	                                 options.BallPivotingCluster,
	                                 options.BallPivotingAngle);

	return res;
}
