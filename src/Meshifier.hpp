#pragma once

#include "Viewer.hpp"

class Meshifier {
public:
	Meshifier(const std::vector<Eigen::Vector3f> & points);

	LIFMesh::Ptr BuildTopMesh(const std::vector<Eigen::Vector3f> & points);

	LIFMesh::Ptr BuildCompleteMesh(const std::vector<Eigen::Vector3f> & points);

private:
	void PopulateTopPoints(const LIFMesh::Ptr & mesh,
	                       const std::vector<Eigen::Vector3f> & points);

	void PopulateBottomPoints(const LIFMesh::Ptr & mesh,
	                          const std::vector<Eigen::Vector3f> & points);

	void PopulateTopFaces(const LIFMesh::Ptr & mesh);

	void PopulateBottomFaces(const LIFMesh::Ptr & mesh, size_t bottomPointOffset);

	void PopulateSideFaces(const LIFMesh::Ptr & ptr, size_t bottomPointOffset);

	std::vector<std::tuple<size_t,size_t,size_t>> d_faces;
	std::vector<size_t>                           d_boundary;
};
