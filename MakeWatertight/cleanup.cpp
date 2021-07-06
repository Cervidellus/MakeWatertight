#include "cleanup.h"
#include "mesh.h"
//#include <stdio.h>
//#include "complex.h"

//#include <cstdint >

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/stat.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/mesh_to_matrix.h>
#include <vcg/complex/algorithms/inertia.h>
#include <vcg/complex/algorithms/update/color.h>

#include <wrap/io_trimesh/export.h>//for debugging

#include <meshlab/src/meshlabplugins/filter_mls/smallcomponentselection.h>

#include <igl/embree/ambient_occlusion.h>

typedef unsigned int uint;
using namespace vcg;
using namespace std;

void Cleanup::makeManifoldAndWatertight(CMeshO& mesh, bool ambientOcclusion, bool removeFolded){
    initialCleanup(mesh);
    
    //updateBoxAndNormals(mesh);
    if (ambientOcclusion)
    {
        Cleanup::ambientOcclusionRemoval(mesh);
    }
    
    bool manifold = false;
    bool watertight = false;
    int iteration = 0;
    while (!manifold && !watertight) {
        iteration++;
        manifold = fixNonManifold(mesh);
        manifold = false;//This prevents leaving while loop until after closeHoles has been called, with a subsequent check on if 2-manifold.
        watertight = closeHoles(mesh);
        if (!watertight) {
            //delete remaining border and folded faces, which can frustrate attempts to close holes. Border should be selected from closeHoles.
            std::cout << "Deleting border faces." << endl;
            int borderFaces = tri::UpdateSelection<CMeshO>::FaceCount(mesh);
            int borderVerts = tri::UpdateSelection<CMeshO>::VertexCount(mesh);
            std::cout << borderVerts << " border vertices and " << borderFaces << " border faces were selected for deletion." << endl;
            std::printf("Mesh has %i vertices and %i triangular faces before deleting border faces.\n", mesh.VN(), mesh.FN());
            deleteSelectedFaces(mesh);
            deleteSelectedVertices(mesh);
            std::printf("Mesh has %i vertices and %i triangular faces after deleting border faces.\n", mesh.VN(), mesh.FN());

            if (removeFolded) {
                selectFoldedFaces(mesh);
                int foldedCount = tri::UpdateSelection<CMeshO>::FaceCount(mesh);
                std::cout << "Deleting " << tri::UpdateSelection<CMeshO>::FaceCount(mesh) << " folded faces." << endl;
                tri::UpdateSelection<CMeshO>::VertexFromFaceLoose(mesh);
                std::cout << tri::UpdateSelection<CMeshO>::FaceCount(mesh) << "faces selected for deletion." << endl;
                std::cout << tri::UpdateSelection<CMeshO>::VertexCount(mesh) << "verteices selected for deletion." << endl;
                deleteSelectedFaces(mesh);
                deleteSelectedVertices(mesh);
                std::printf("Mesh has %i vertices and %i triangular faces after deleting border faces and folded faces.\n", mesh.VN(), mesh.FN());
            }



            //selectBorder(mesh);
            //tri::SmallComponent<CMeshO>::DeleteFaceVert(mesh);
            deleteSmallDisconnectedComponent(mesh);//Not working?

            int result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
            std::cout << result << " unreferenced vertices removed." << endl;

            tri::UpdateTopology<CMeshO>::FaceFace(mesh);
            tri::UpdateTopology<CMeshO>::VertexFace(mesh);
            tri::Allocator<CMeshO>::CompactVertexVector(mesh);
            tri::Allocator<CMeshO>::CompactFaceVector(mesh);

            if (iteration == 3) {
                cout << "saving iteration 3" << endl; 
                vcg::tri::io::Exporter<CMeshO>::Save(mesh, "E:/OneDrive/src/MakeWatertight/MakeWatertight/output/iteration3.ply", vcg::tri::io::Mask::IOM_VERTCOLOR | vcg::tri::io::Mask::IOM_VERTQUALITY);
            }
        }
    }
    deleteSmallDisconnectedComponent(mesh);
}

void Cleanup::initialCleanup(CMeshO& mesh) {
    int result = 0;
    deleteSmallDisconnectedComponent(mesh);
    std::cout << " Deleted small disconnected component." << endl;
    result = tri::Clean<CMeshO>::RemoveFaceOutOfRangeArea(mesh, 0);//This I believe causes some unfortunate holes and t-vertices.
    std::cout << result << " Zero area vertices removed." << endl;

    result = tri::Clean<CMeshO>::RemoveDuplicateFace(mesh);
    std::cout << result << " Duplicate Faces removed." << endl;

    result = tri::Clean<CMeshO>::RemoveDuplicateVertex(mesh);
    std::cout << result << " duplicate vertices removed." << endl;
    if (result != 0) updateBoxAndNormals(mesh);

    result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
    std::cout << result << " unreferenced vertices removed." << endl;
    if (result != 0) updateBoxAndNormals(mesh);
}

void Cleanup::ambientOcclusionRemoval(CMeshO& mesh) {
    //Step 1: Get the matrices from the meshlab mesh.
    int verts_deleted;
    CMeshO::VertexIterator vi;
    CMeshO::FaceIterator fi;

    Eigen::MatrixXf V;
    Eigen::MatrixXi F;
    Eigen::MatrixXf NV; //Vertex Normals
    Eigen::MatrixXf NF; //Face normals
    Eigen::VectorXf AO; //Ambient Occlusion Values


    tri::Allocator<CMeshO>::CompactVertexVector(mesh);
    tri::Allocator<CMeshO>::CompactFaceVector(mesh);
    tri::MeshToMatrix<CMeshO>::GetTriMeshData(mesh,F,V);
    tri::MeshToMatrix<CMeshO>::GetNormalData(mesh, NV, NF);

    //Step two: Use libigl to calculate the ambient occlusion.
    //TODO::see if there is a time cost to more rays (i.e. maybe instead of 500, 1000)
    std::cout << "Calculating ambient occlusion." << endl;

    igl::embree::ambient_occlusion(V,F,V,NV,500,AO);

    tri::UpdateTopology<CMeshO>::VertexFace(mesh);//FOR DEBUG

    std::cout << "Applying ambient occlusion." << endl;
    for (vi = mesh.vert.begin(); vi != mesh.vert.end(); ++vi)
        if (!(*vi).IsD())
        {
            vi->Q() = AO[vi->Index()];
        }

    //Step four: Delete faces and vertices based on that quality.
    //Modeled after meshlab FP_SELECT_DELETE_FACEVERT, with the exception that I select based on quality 

    tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    tri::UpdateSelection<CMeshO>::VertexFromQualityRange(mesh, 0.95, 1);
    //tri::UpdateSelection<CMeshO>::FaceFromVertexStrict(mesh);
    //deleteSelectedFacesAndVerts(mesh);//crashes here
    deleteSelectedVertices(mesh);

    std::printf("Mesh has %i vertices and %i triangular faces after ambient occlusion removal.\n", mesh.VN(), mesh.FN());

    tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);

    vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
    tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    tri::Allocator<CMeshO>::CompactVertexVector(mesh);
    std::printf("Mesh has %i vertices and %i triangular faces after cleanup.\n", mesh.VN(), mesh.FN());//why is this low?
    tri::Allocator<CMeshO>::CompactFaceVector(mesh);

    deleteSmallDisconnectedComponent(mesh);
}

bool Cleanup::fixNonManifold(CMeshO& mesh) {
    bool manifold = false;
    int iteration = 0;
    int result = 0;

    while (!manifold) {
        tri::UpdateSelection<CMeshO>::VertexClear(mesh);
        tri::UpdateSelection<CMeshO>::FaceClear(mesh);
        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        tri::Allocator<CMeshO>::CompactVertexVector(mesh);
        tri::Allocator<CMeshO>::CompactFaceVector(mesh);
        updateBoxAndNormals(mesh);

        if ((tri::Clean<CMeshO>::CountNonManifoldVertexFF(mesh) == 0) && (tri::Clean<CMeshO>::CountNonManifoldEdgeFF(mesh) == 0))
        {
            if (iteration == 0) std::cout << "No non-manifold geometries to fix." << endl;
            else std::cout << "Mesh is 2-manifold after iteration:" << iteration << endl;
            manifold = true;
        }
        else {
            iteration++;
            std::cout << endl << "Beginning iteration " << iteration << " of fixNonManifold" << endl;
            std::printf("Mesh has %i vertices and %i triangular faces\n", mesh.VN(), mesh.FN());

            //Remove incident faces to fix non-manifold edges.
            result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
            std::cout << result << " unreferenced vertices removed." << endl;
            //std::printf("Mesh has %i vertices and %i triangular faces\n", mesh.VN(), mesh.FN());
            if (result != 0) updateBoxAndNormals(mesh);
            result = tri::Clean<CMeshO>::RemoveNonManifoldFace(mesh);
            std::cout << result << " incident faces removed to fix non-manifold edges." << endl;
            if (result != 0) updateBoxAndNormals(mesh);
            tri::UpdateTopology<CMeshO>::FaceFace(mesh);
            tri::UpdateTopology<CMeshO>::VertexFace(mesh);
            tri::Allocator<CMeshO>::CompactVertexVector(mesh);
            tri::Allocator<CMeshO>::CompactFaceVector(mesh);

            //Try and fix remainder by splitting vertices. This eats up a ton of memory, and I may want to rewrite.
            //cout << "Repairing non-manifold edges through vertex splitting." << endl;
            //std::printf("Mesh has %i vertices and %i triangular faces after update.\n", mesh.VN(), mesh.FN());
            //requirements MeshModel::MM_FACEFACETOPO | MeshModel::MM_VERTMARK;
            //Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE
            result = tri::Clean<CMeshO>::SplitManifoldComponents(mesh/*, .005*/);//I need to add a move threshold
            std::cout << result << " vertices split by SplitManifoldComponents." << endl;
            //This results in a crazy number of unreferenced vertices! Is this a bug?
            std::printf("Mesh has %i vertices and %i triangular faces post vertex splitting. \n", mesh.VN(), mesh.FN());
            result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
            std::cout << result << " unreferenced vertices removed." << endl;
            //std::printf("Mesh has %i vertices and %i triangular faces\n", mesh.VN(), mesh.FN());
            if (result != 0) updateBoxAndNormals(mesh);
            tri::UpdateTopology<CMeshO>::FaceFace(mesh);
            tri::UpdateTopology<CMeshO>::VertexFace(mesh);
            tri::Allocator<CMeshO>::CompactVertexVector(mesh);
            tri::Allocator<CMeshO>::CompactFaceVector(mesh);

            //Fix non-manifold vertices by vertex splitting.
            std::cout << "Repairing non-manifold vertices by vertex splitting." << endl;
            result = tri::Clean<CMeshO>::SplitNonManifoldVertex(mesh, .005);
            std::cout << result << " vertices split by SplitManifoldVertex." << endl;
            std::printf("Mesh has %i vertices and %i triangular faces post vertex splitting. \n", mesh.VN(), mesh.FN());
            if (result != 0) updateBoxAndNormals(mesh);
            result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
            std::cout << result << " unreferenced vertices removed." << endl;
            std::printf("Mesh has %i vertices and %i triangular faces\n", mesh.VN(), mesh.FN());
            if (result != 0) updateBoxAndNormals(mesh);

            //select and delete remaining non-manifold vertices.
            tri::UpdateSelection<CMeshO>::VertexClear(mesh);
            int nonManifoldVerts = tri::Clean<CMeshO>::CountNonManifoldVertexFF(mesh);
            if (nonManifoldVerts > 0)
            {
                //int result = deleteSelectedVerts(mesh);
                std::cout << "Deleting remaining non-manifold vertices." << endl;
                deleteSelectedVertices(mesh);
                std::cout << "Verts, faces post deletion:" << mesh.VN() << "," << mesh.FN() << endl;
            }
        }
    }
    return manifold;
}

//This immitates Meshlab. Defined in meshmodel.cpp
void Cleanup::updateBoxAndNormals(CMeshO& mesh) {
    tri::UpdateBounding<CMeshO>::Box(mesh);

    //meshlab uses mesh.fn instead of mesh.FN(), though it tells you never to use fn
    if (mesh.FN() > 0) {
        mesh.vert.EnableNormal();
        //mesh.face.EnableNormal();
        tri::UpdateNormal<CMeshO>::PerFaceNormalized(mesh);
        tri::UpdateNormal<CMeshO>::PerVertexAngleWeighted(mesh);//crash here after AO?
    }
}

int Cleanup::deleteSelectedFacesAndVerts(CMeshO& mesh) {
    //Flag as deleted all vertices that are flagged as selected.
    //Might be more effecient for me to re-write CountNonManifoldVertexFF and flag as delete instead of selected.
    cout << tri::UpdateSelection<CMeshO>::VertexCount(mesh) << " vertices selected at start of delteSelectedFacesAndVerts" << endl;
    CMeshO::FaceIterator faceIterator;
    CMeshO::VertexIterator vertexIterator;
    int deleted = 0;
    //Some of this updating is likely excessive.. should go through and test what I can remove:
    tri::Allocator<CMeshO>::CompactVertexVector(mesh);
    tri::Allocator<CMeshO>::CompactFaceVector(mesh);
    mesh.face.EnableFFAdjacency();
    mesh.vert.EnableVFAdjacency();
    mesh.face.EnableVFAdjacency();
    tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    tri::UpdateTopology<CMeshO>::VertexFace(mesh);
    cout << tri::UpdateSelection<CMeshO>::VertexCount(mesh) << " vertices after updating in  delteSelectedFacesAndVerts" << endl;
    int vvn = mesh.vn;
    int ffn = mesh.fn;

    tri::UpdateSelection<CMeshO>::VertexFromFaceStrict(mesh);//strict gets rid of some of what we want. Loose crashes after ambient occlusion. Crashes hard if I get rid of it. 
    cout << tri::UpdateSelection<CMeshO>::VertexCount(mesh) << " vertices selectedafter vertexfromfaceLo" << endl;
    for (faceIterator = mesh.face.begin(); faceIterator != mesh.face.end(); ++faceIterator)
        if (!(*faceIterator).IsD() && (*faceIterator).IsS())
            tri::Allocator<CMeshO>::DeleteFace(mesh, *faceIterator);
    for (vertexIterator = mesh.vert.begin(); vertexIterator != mesh.vert.end(); ++vertexIterator)
        if (!(*vertexIterator).IsD() && (*vertexIterator).IsS())
        {
            tri::Allocator<CMeshO>::DeleteVertex(mesh, *vertexIterator);
            deleted++;
        }

    updateBoxAndNormals(mesh);
    std::printf("Deleted %i vertices, %i faces.\n", vvn - mesh.vn, ffn - mesh.fn);

    return deleted;
}






int Cleanup::deleteSelectedFaces(CMeshO& mesh) {
    int deleted = 0;

    if (tri::UpdateSelection<CMeshO>::FaceCount(mesh) > 0)
    {
        CMeshO::FaceIterator faceIterator;
        for (faceIterator = mesh.face.begin(); faceIterator != mesh.face.end(); ++faceIterator)
            if (!(*faceIterator).IsD() && (*faceIterator).IsS()) {
                tri::Allocator<CMeshO>::DeleteFace(mesh, *faceIterator);
                deleted++;
            }
                
    }

    return deleted;
}


int Cleanup::deleteSelectedVertices(CMeshO& mesh) {
    int deleted = 0;

    if (tri::UpdateSelection<CMeshO>::VertexCount(mesh) > 0)
    {
        tri::UpdateSelection<CMeshO>::FaceClear(mesh);
        tri::UpdateSelection<CMeshO>::FaceFromVertexLoose(mesh);
        CMeshO::FaceIterator faceIterator;
        CMeshO::VertexIterator vertexIterator;

        for (faceIterator = mesh.face.begin(); faceIterator != mesh.face.end(); ++faceIterator)
            if (!(*faceIterator).IsD() && (*faceIterator).IsS())
                tri::Allocator<CMeshO>::DeleteFace(mesh, *faceIterator);
        for (vertexIterator = mesh.vert.begin(); vertexIterator != mesh.vert.end(); ++vertexIterator)
            if (!(*vertexIterator).IsD() && (*vertexIterator).IsS()) {
                tri::Allocator<CMeshO>::DeleteVertex(mesh, *vertexIterator);
                deleted++;
            }
                
        updateBoxAndNormals(mesh);
    }
    return deleted;
}




bool Cleanup::closeHoles(CMeshO& mesh, int maxHoleSize, bool selected, bool avoidSelfIntersection) {
    std::cout << "Closing holes." << endl;
    bool closed = false;

    //first try filling the hole.
    tri::Hole<CMeshO>::EarCuttingIntersectionFill<tri::SelfIntersectionEar< CMeshO> >(mesh, maxHoleSize, selected);

    tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    selectBorder(mesh);
    //int borderVerts = tri::UpdateSelection<CMeshO>::VertexCount(mesh);
    int borderFaces = tri::UpdateSelection<CMeshO>::FaceCount(mesh);
    //cout << borderVerts << " border vertices and " << borderFaces << " found after closing holes." << endl;
    if (borderFaces == 0) closed = true;
           
    return closed;
}

void Cleanup::selectBorder(CMeshO& mesh)
{
    //tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    //tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    tri::Allocator<CMeshO>::CompactVertexVector(mesh);
    tri::Allocator<CMeshO>::CompactFaceVector(mesh);
    tri::UpdateFlags<CMeshO>::FaceBorderFromNone(mesh);
    tri::UpdateFlags<CMeshO>::VertexBorderFromFaceBorder(mesh);
    tri::UpdateSelection<CMeshO>::FaceFromBorderFlag(mesh);
    tri::UpdateSelection<CMeshO>::VertexFromBorderFlag(mesh);
}

void Cleanup::selectFoldedFaces(CMeshO& mesh)
{
    //Temp for debug:
    cout << "Selected faces at start of selectFoldedFaces:" << tri::UpdateSelection<CMeshO>::FaceCount(mesh) << endl;

    //tri::Allocator<CMeshO>::CompactVertexVector(mesh);
    //tri::Allocator<CMeshO>::CompactFaceVector(mesh);
    ////tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    ////tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    //tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    //tri::UpdateTopology<CMeshO>::VertexFace(mesh);
    tri::Clean<CMeshO>::SelectFoldedFaceFromOneRingFaces(mesh, 2.9);//2.9 is roughly 170 degrees.

        //Temp for debug:
    cout << "Selected faces at end of selectFoldedFaces:" << tri::UpdateSelection<CMeshO>::FaceCount(mesh);
}

void Cleanup::deleteSmallDisconnectedComponent(CMeshO& mesh) {
    vcg::tri::SmallComponent<CMeshO>::Select(mesh);
    vcg::tri::UpdateSelection<CMeshO>::VertexFromFaceLoose(mesh);//TODO:shouldn't this be loose?
    cout << "Selected Faces after selecting small component:" << tri::UpdateSelection<CMeshO>::FaceCount(mesh) << endl;
    //cout << "Selected Vertices after selecting small component:" << tri::UpdateSelection<CMeshO>::VertexCount(mesh) << endl;//selects only vertices.
    //deleteSelectedFacesAndVerts(mesh);
    //deleteSelectedFaces(mesh)
    deleteSelectedVertices(mesh);
}
