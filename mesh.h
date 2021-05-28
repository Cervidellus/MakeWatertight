#ifndef MESH_H
#define MESH_H
#include<vcg/complex/complex.h>

//this is the type from meshlab. I've commented out what I am not using.
class CVertexO;
class CEdgeO;
class CFaceO;

// Declaration of the semantic of the used types
class CUsedTypesO: public vcg::UsedTypes < vcg::Use<CVertexO>::AsVertexType,
        vcg::Use<CEdgeO   >::AsEdgeType,
        vcg::Use<CFaceO  >::AsFaceType >{};


// The Main Vertex Class
// Most of the attributes are optional and must be enabled before use.
// Each vertex needs 40 byte, on 32bit arch. and 44 byte on 64bit arch.

class CVertexO  : public vcg::Vertex< CUsedTypesO,
        vcg::vertex::InfoOcf,           /*  4b */
        vcg::vertex::Coord3f,           /* 12b */
        vcg::vertex::BitFlags,          /*  4b */
//        vcg::vertex::Normal3f,          /* 12b *///THey use Normal3m, which is their own?
        vcg::vertex::Normal3fOcf,
        vcg::vertex::Qualityf,          /*  4b */

//        vcg::vertex::Color4b,           /*  4b */
        vcg::vertex::VFAdjOcf,          /*  0b */
        vcg::vertex::MarkOcf           /*  0b */
//        vcg::vertex::TexCoordfOcf,      /*  0b */
//        vcg::vertex::CurvaturefOcf,     /*  0b */
//        vcg::vertex::CurvatureDirfOcf,  /*  0b */
//        vcg::vertex::RadiusfOcf         /*  0b */
        >{
};


// The Main Edge Class
class CEdgeO : public vcg::Edge<CUsedTypesO,
        vcg::edge::BitFlags,          /*  4b */
        vcg::edge::EVAdj,
        vcg::edge::EEAdj
        >{
};

// Each face needs 32 byte, on 32bit arch. and 48 byte on 64bit arch.
class CFaceO    : public vcg::Face<  CUsedTypesO,
        vcg::face::InfoOcf,              /* 4b */
        vcg::face::VertexRef,            /*12b */
        vcg::face::BitFlags,             /* 4b */
//        vcg::face::Normal3f,             /*12b */
        vcg::face::Normal3fOcf,
//        vcg::face::QualityfOcf,          /* 0b */
//        vcg::face::MarkOcf,              /* 0b */
//        vcg::face::Color4bOcf,           /* 0b */
        vcg::face::FFAdjOcf,             /* 0b */
        vcg::face::VFAdjOcf           /* 0b */
//        vcg::face::CurvatureDirfOcf,     /* 0b */
//        vcg::face::WedgeTexCoordfOcf     /* 0b */
        > {};

typedef vcg::tri::TriMesh< vcg::vertex::vector_ocf<CVertexO>, vcg::face::vector_ocf<CFaceO> > vcgTriMesh;

//simplified here for our purposes:

class CMeshO : public vcg::tri::TriMesh< vcg::vertex::vector_ocf<CVertexO>, vcg::face::vector_ocf<CFaceO> > {};

#endif // MESH_H
