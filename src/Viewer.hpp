#pragma once

#include <QGLFunctions>
#include <qglviewer.h>

#include <memory>

#include <vcg/complex/complex.h>

class LIFFace;
class LIFVertex;
struct LIFUsedTypes : public vcg::UsedTypes<vcg::Use<LIFVertex>::AsVertexType,
                                            vcg::Use<LIFFace>::AsFaceType>{};
class LIFVertex  : public vcg::Vertex<LIFUsedTypes,
                                      vcg::vertex::Coord3f,
                                      vcg::vertex::Normal3f,
                                      vcg::vertex::BitFlags>{};
class LIFFace    : public vcg::Face<LIFUsedTypes,
                                    vcg::face::VertexRef,
                                    vcg::face::Normal3f,
                                    vcg::face::FFAdj,
                                    vcg::face::BitFlags > {};

class LIFMesh    : public vcg::tri::TriMesh<std::vector<LIFVertex>,
                                            std::vector<LIFFace>> {
public:
	typedef std::shared_ptr<LIFMesh> Ptr;
	static Ptr Dodecahedron();
};





class Viewer : public QGLViewer , protected QGLFunctions {
	Q_OBJECT
public:
	Viewer(QWidget * parent = nullptr);
	virtual ~Viewer();

	void setMesh(const LIFMesh::Ptr & model);

	void draw() override;

	void init() override;

	QString helpString() const override;
private :
	GLuint d_vaoID,d_vboID,d_nboID,d_cboID;
	size_t d_size;
	bool   d_ready;
	std::shared_ptr<LIFMesh> d_onLoad;

};
