#include "Viewer.hpp"

#include <chrono>

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
	setAxisIsDrawn(true);

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
		setModel(*d_onLoad);
		d_onLoad.reset();
	}
}

QString Viewer::helpString() const {

}


void Viewer::setModel(const Model & model) {
	if ( d_ready == false ) {
		d_onLoad = std::make_shared<Model>(model);
		std::cerr << "Not initialized" << std::endl;
		return;
	}
	using clock = std::chrono::high_resolution_clock;
	auto start = clock::now();

	std::vector<GLfloat> vertices,normals,colors;
	vertices.reserve(3*model.VertexIndices.size());
	normals.reserve(3*model.VertexIndices.size());
	for ( const auto & vIdx : model.VertexIndices ) {
		vertices.push_back(model.Vertices[3*vIdx + 0]);
		vertices.push_back(model.Vertices[3*vIdx + 1]);
		vertices.push_back(model.Vertices[3*vIdx + 2]);
		colors.push_back(0.0);
		colors.push_back(0.7f);
		colors.push_back(0.7f);
	}

	for ( const auto & nIdx : model.NormalIndices ) {
		normals.push_back(model.Normals[3*nIdx + 0]);
		normals.push_back(model.Normals[3*nIdx + 1]);
		normals.push_back(model.Normals[3*nIdx + 2]);
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
}



Model Model::Cube() {
	Model res;
	res.Vertices = {
	                1.0,1.0,1.0,
	                1.0,-1.0,1.0,
	                -1.0,-1.0,1.0,
	                -1.0,1.0,1.0,
	                1.0,1.0,-1.0,
	                1.0,-1.0,-1.0,
	                -1.0,-1.0,-1.0,
	                -1.0,1.0,-1.0,
	};
	res.VertexIndices = {
	                     0,1,2,2,0,3,
	                     4,5,6,4,6,7,
	                     0,1,5,4,5,0,
	                     2,3,6,3,6,7,
	                     0,3,4,3,4,7,
	                     1,2,5,2,5,6
	};

	res.Normals = { 0,0,1,
	                0,1,0,
	                1,0,0,
	                0,0,-1,
	                0,-1,0,
	                -1,0,0
	};
	res.NormalIndices = {
	                     0,0,0,0,0,0,
	                     3,3,3,3,3,3,
	                     2,2,2,2,2,2,
	                     5,5,5,5,5,5,
	                     1,1,1,1,1,1,
	                     4,4,4,4,4,4
	};
	return res;
}
