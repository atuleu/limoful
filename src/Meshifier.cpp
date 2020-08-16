#include "Meshifier.hpp"

#include <vcg/complex/algorithms/update/normal.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>


Meshifier::Meshifier(const std::vector<Eigen::Vector3f> & points) {
	typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;
	typedef CGAL::Triangulation_vertex_base_with_info_2<size_t, K> Vb;
	typedef CGAL::Triangulation_data_structure_2<Vb>               Tds;
	typedef CGAL::Delaunay_triangulation_2<K,Tds>                  Delaunay;
	typedef K::Point_2                                             Point2;

	std::vector<std::pair<Point2,size_t>> pointDTs;
	pointDTs.reserve(points.size());
	size_t i = -1;
	for ( const auto & p : points ) {
		pointDTs.push_back(std::make_pair(Point2(p.x(),p.y()),
		                                  ++i));
	}

	Delaunay dt;
	dt.insert(pointDTs.begin(),pointDTs.end());
	d_faces.clear();
	for ( auto faceIter = dt.finite_faces_begin();
	      faceIter != dt.finite_faces_end();
	      ++faceIter ) {
		d_faces.push_back({faceIter->vertex(0)->info(),
		                   faceIter->vertex(1)->info(),
		                   faceIter->vertex(2)->info()});

	}


	auto vIter = dt.incident_vertices(dt.infinite_vertex());
	auto end = vIter;
	do {
		d_boundary.push_back(vIter->info());
	} while(++vIter != end);
	//repeat the first one.
	d_boundary.push_back(vIter->info());
}


LIFMesh::Ptr Meshifier::BuildTopMesh(const std::vector<Eigen::Vector3f> & points) {
	auto mesh = std::make_shared<LIFMesh>();
	PopulateTopPoints(mesh,points);
	PopulateTopFaces(mesh);
	vcg::tri::UpdateNormal<LIFMesh>::PerFaceNormalized(*mesh);
	return mesh;
}

LIFMesh::Ptr Meshifier::BuildCompleteMesh(const std::vector<Eigen::Vector3f> & points) {
	auto mesh = std::make_shared<LIFMesh>();
	PopulateTopPoints(mesh,points);
	PopulateBottomPoints(mesh,points);
	PopulateTopFaces(mesh);
	PopulateBottomFaces(mesh,points.size());
	PopulateSideFaces(mesh,points.size());
	vcg::tri::UpdateNormal<LIFMesh>::PerFaceNormalized(*mesh);
	return mesh;
}

void Meshifier::PopulateTopPoints(const LIFMesh::Ptr & mesh,
                                  const std::vector<Eigen::Vector3f> & points) {
	for ( const auto & p : points ) {
		vcg::tri::Allocator<LIFMesh>::AddVertex(*mesh,LIFMesh::CoordType(p.x(),
		                                                                 p.y(),
		                                                                 p.z()));

	}
}

void Meshifier::PopulateBottomPoints(const LIFMesh::Ptr & mesh,
                                     const std::vector<Eigen::Vector3f> & points) {
	for ( const auto & p : points ) {
		vcg::tri::Allocator<LIFMesh>::AddVertex(*mesh,LIFMesh::CoordType(p.x(),
		                                                                 p.y(),
		                                                                 0));

	}
}


void Meshifier::PopulateTopFaces(const LIFMesh::Ptr & mesh) {
	for ( const auto & f : d_faces ) {
		vcg::tri::Allocator<LIFMesh>::AddFace(*mesh,
		                                      std::get<0>(f),
		                                      std::get<1>(f),
		                                      std::get<2>(f));
	}
}

void Meshifier::PopulateBottomFaces(const LIFMesh::Ptr & mesh,
                                    size_t bottomPointOffset) {
	for ( const auto & f : d_faces ) {
		// we reverse order for bottom side
		vcg::tri::Allocator<LIFMesh>::AddFace(*mesh,
		                                      std::get<2>(f) + bottomPointOffset,
		                                      std::get<1>(f) + bottomPointOffset,
		                                      std::get<0>(f) + bottomPointOffset);
	}
}


void Meshifier::PopulateSideFaces(const LIFMesh::Ptr & mesh,
                                  size_t bottomPointOffset) {
	for ( auto bIter = d_boundary.cbegin();
	      bIter != (d_boundary.cend() - 1);
	      ++bIter ) {
		vcg::tri::Allocator<LIFMesh>::AddFace(*mesh,
		                                      *bIter,
		                                      *(bIter+1),
		                                      *(bIter+1) + bottomPointOffset);

		vcg::tri::Allocator<LIFMesh>::AddFace(*mesh,
		                                      *(bIter+1) + bottomPointOffset,
		                                      *bIter + bottomPointOffset,
		                                      *bIter);
	}
}
