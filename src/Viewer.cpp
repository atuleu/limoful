#include "Viewer.hpp"

#include <chrono>

#include<vcg/complex/algorithms/create/platonic.h>

Viewer::Viewer(QWidget * parent)
	: QGLViewer(parent)
	, d_size(0)
	, d_ready(false) {
}

Viewer::~Viewer() {
}

void Viewer::draw() {
	if ( d_size == 0 ) {
		return;
	}

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, d_vboID);

	glVertexAttribPointer(0,                  // attribute
	                      3,                  // size
	                      GL_FLOAT,           // type
	                      GL_FALSE,           // normalized?
	                      0,                  // stride
	                      (void*)0            // array buffer offset
	                      );

	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, d_cboID);
	glVertexAttribPointer(1,                                // attribute
	                      2,                                // size
	                      GL_FLOAT,                         // type
	                      GL_FALSE,                         // normalized?
	                      0,                                // stride
	                      (void*)0                          // array buffer offset
	                      );


	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, d_nboID);
	glVertexAttribPointer(2,                                // attribute
	                      3,                                // size
	                      GL_FLOAT,                         // type
	                      GL_FALSE,                         // normalized?
	                      0,                                // stride
	                      (void*)0                          // array buffer offset
	                      );

	glDrawArrays(GL_TRIANGLES,0,d_size);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
}

void Viewer::init() {
	initializeGLFunctions();

	setGridIsDrawn(true);
	setAxisIsDrawn(false);

	//glGenVertexArrays(1, &d_vaoID);
	//glBindVertexArray(d_vaoID);

	glGenBuffers(1,&d_vboID);
	glGenBuffers(1,&d_nboID);
	glGenBuffers(1,&d_cboID);


	camera()->setUpVector({0,0,1});

	camera()->setViewDirection({1,1,-1});
	setSceneRadius(1.5);
	showEntireScene();

	d_ready = true;
	if ( d_onLoad ) {
		setMesh(d_onLoad);
		d_onLoad.reset();
	}
}

QString Viewer::helpString() const {

}


void Viewer::setMesh(const LIFMesh::Ptr & mesh) {
	if ( d_ready == false ) {
		d_onLoad = mesh;
		return;
	}
	using clock = std::chrono::high_resolution_clock;
	auto start = clock::now();

	std::vector<GLfloat> vertices,normals,colors;
	vertices.reserve(3*mesh->face.size());
	normals.reserve(3*mesh->face.size());
	for ( const auto & face : mesh->face ) {
		const auto & n = face.cN();
		for ( size_t i = 0; i < 3; ++i ) {
			const auto & p = face.cP(i);
			vertices.push_back(p[0]);
			vertices.push_back(p[1]);
			vertices.push_back(p[2]);
			normals.push_back(n[0]);
			normals.push_back(n[1]);
			normals.push_back(n[2]);
			colors.push_back(0);
			colors.push_back(0.7);
			colors.push_back(0.7);
		}
	}

	glBindBuffer(GL_ARRAY_BUFFER,d_vboID);
	glBufferData(GL_ARRAY_BUFFER,sizeof(GLfloat)*vertices.size(),&vertices[0],GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER,d_nboID);
	glBufferData(GL_ARRAY_BUFFER,sizeof(GLfloat)*normals.size(),&normals[0],GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER,d_cboID);
	glBufferData(GL_ARRAY_BUFFER,sizeof(GLfloat)*colors.size(),&colors[0],GL_STATIC_DRAW);



	d_size = std::min(vertices.size(),normals.size());
	std::chrono::duration<float,std::milli> ellapsed = clock::now() - start;
	std::cerr << d_size << " vertices" << " and " << d_size / 3  <<  " triangles loaded in "
	          << ellapsed.count() << "ms." << std::endl;

	update();
}



LIFMesh::Ptr LIFMesh::Dodecahedron() {
	auto mesh = std::make_shared<LIFMesh>();
	vcg::tri::Dodecahedron(*mesh);
	vcg::tri::UpdateNormal<LIFMesh>::PerFaceNormalized(*mesh);
	return mesh;
}
