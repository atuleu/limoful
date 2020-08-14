#pragma once

#include <QGLFunctions>
#include <qglviewer.h>

struct Model {
	std::vector<float>  Vertices;
	std::vector<float>  Normals;
	std::vector<size_t> VertexIndices,NormalIndices;

	static Model Cube();
};


class Viewer : public QGLViewer , protected QGLFunctions {
	Q_OBJECT
public:
	Viewer(QWidget * parent = nullptr);
	virtual ~Viewer();

	void setModel(const Model & model);


	void draw() override;

	void init() override;

	QString helpString() const override;
private :
	GLuint d_vaoID,d_vboID,d_nboID,d_cboID;
	size_t d_size;
};
