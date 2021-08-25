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
//#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>

#include <wrap/io_trimesh/export.h>//for debugging

#include <meshlab/src/meshlabplugins/filter_mls/smallcomponentselection.h>
#include <meshlab/src/meshlabplugins/filter_meshing/quadric_simp.h>

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
        int result;
        iteration++;
        manifold = fixNonManifold(mesh);
        //Fix zero area faces by removing, then merging edges.
        //result = tri::Clean<CMeshO>::RemoveFaceOutOfRangeArea(mesh, 0);//instead of this, fix t-vertices edge collapse
        //if (result > 0)
        //{
        //vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        //vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        //tri::Allocator<CMeshO>::CompactVertexVector(mesh);
        //tri::Allocator<CMeshO>::CompactFaceVector(mesh);
        //updateBoxAndNormals(mesh);
        mesh.face.DisableFFAdjacency();
        mesh.face.DisableVFAdjacency();
        result = tri::Clean<CMeshO>::RemoveTVertexByCollapse(mesh, 20, true);//crash here
        cout << result << " T-vertices repaired by edge collapse." << endl;
        mesh.face.EnableFFAdjacency();
        mesh.face.EnableVFAdjacency();
        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        result = tri::Clean<CMeshO>::RemoveNonManifoldFace(mesh);//crash Assertion failed: f.cFFp(j) != 0, file E:\OneDrive\src\vcglib_dev\vcg\simplex\face\topology.h, line 43
        //maybe not needed to update:
        cout << result << " resulting incident faces removed." << endl;
        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        tri::Allocator<CMeshO>::CompactVertexVector(mesh);
        tri::Allocator<CMeshO>::CompactFaceVector(mesh);
        updateBoxAndNormals(mesh);
        //}

        manifold = false;//This prevents leaving while loop until after closeHoles has been called, with a subsequent check on if 2-manifold.
        watertight = closeHoles(mesh); //This can cause zero-area faces to show up again. Can I check for this?
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

            deleteSmallDisconnectedComponent(mesh);

            result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
            std::cout << result << " unreferenced vertices removed." << endl;

            tri::UpdateTopology<CMeshO>::FaceFace(mesh);
            tri::UpdateTopology<CMeshO>::VertexFace(mesh);
            tri::Allocator<CMeshO>::CompactVertexVector(mesh);
            tri::Allocator<CMeshO>::CompactFaceVector(mesh);

            //for testing, will be removed.
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
    //result = tri::Clean<CMeshO>::RemoveFaceOutOfRangeArea(mesh, 0);//This I believe causes some unfortunate holes and t-vertices.
    //std::cout << result << " Zero area vertices removed." << endl;

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
    int borderFaces = tri::UpdateSelection<CMeshO>::FaceCount(mesh);
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
    tri::Clean<CMeshO>::SelectFoldedFaceFromOneRingFaces(mesh, 2.9);//2.9 is roughly 170 degrees.
}

void Cleanup::deleteSmallDisconnectedComponent(CMeshO& mesh) {
    vcg::tri::SmallComponent<CMeshO>::Select(mesh);
    vcg::tri::UpdateSelection<CMeshO>::VertexFromFaceLoose(mesh);
    cout << "Selected Faces after selecting small component:" << tri::UpdateSelection<CMeshO>::FaceCount(mesh) << endl;

    deleteSelectedVertices(mesh);
}

//void Cleanup::decimate(CMeshO& mesh, double ratio) {
//    //m.updateDataMask(MeshModel::MM_VERTFACETOPO | MeshModel::MM_VERTMARK);
//    mesh.vert.EnableVFAdjacency();
//    mesh.face.EnableVFAdjacency();
//    mesh.vert.EnableMark();
//    tri::UpdateTopology<CMeshO>::VertexFace(mesh);
//
//    tri::UpdateFlags<CMeshO>::FaceBorderFromVF(mesh);
//
//    //int TargetFaceNum = par.getInt("TargetFaceNum");
//    int TargetFaceNum = mesh.fn * ratio;
//
//    tri::TriEdgeCollapseQuadricParameter pp;
//    //pp.QualityThr = lastq_QualityThr = par.getFloat("QualityThr");
//    pp.PreserveBoundary = false;
//    //pp.FastPreserveBoundary = true;
//    pp.BoundaryQuadricWeight = 1;
//    pp.PreserveTopology = true;
//    pp.QualityWeight = false;
//    pp.NormalCheck = false;
//    pp.OptimalPlacement = true;
//    pp.QualityQuadric = false;
//    pp.QualityQuadricWeight =.001;
//    bool lastq_Selected = false;
//    //vcg::CallBackPos cb;
//    //QuadricSimplification(mesh, TargetFaceNum, lastq_Selected, pp, cb);//instead of calling this, which requires a callback, copy code from quadric_simp.cpp
//
//
//    math::Quadric<double> QZero;
//    QZero.SetZero();
//
//    tri::QuadricTemp TD(mesh.vert, QZero);
//    //tri::QHelper::TDp() = &TD;
//
//    
//
//
//
//
//
//
//
//    vcg::LocalOptimization<CMeshO> DeciSession(mesh, &pp);
//    //cb(1, "Initializing simplification");
//
//
//
//
//    // this fails to compile. I should really wro around mytriedgecollapse, and directly use the one from vcg
//
//    DeciSession.Init<tri::MyTriEdgeCollapse >();
//
//
//
//
//
// /*   if (Selected)
//        TargetFaceNum = m.fn - (m.sfn - TargetFaceNum);*/
//    DeciSession.SetTargetSimplices(TargetFaceNum);
//    DeciSession.SetTimeBudget(0.1f); // this allows updating the progress bar 10 time for sec...
//    //  if(TargetError< numeric_limits<double>::max() ) DeciSession.SetTargetMetric(TargetError);
//    //int startFn=m.fn;
//    int faceToDel = mesh.fn - TargetFaceNum;
//    cout << "Decimating mesh" << endl;
//    while (DeciSession.DoOptimization() && mesh.fn > TargetFaceNum)
//    {
//        int percentDone = 100 - 100 * (mesh.fn - TargetFaceNum) / (faceToDel);
//        if (percentDone%10 == 0) {
//            cout << percentDone << " percent finished." << endl;
//        }
//        //cb(100 - 100 * (m.fn - TargetFaceNum) / (faceToDel), "Simplifying...");
//    };
//
//    DeciSession.Finalize<tri::MyTriEdgeCollapse >();
//
//    //if (Selected) // Clear Writable flags 
//    //{
//    //    for (auto vi = m.vert.begin(); vi != m.vert.end(); ++vi)
//    //    {
//    //        if (!(*vi).IsD()) (*vi).SetW();
//    //        if ((*vi).IsS()) (*vi).ClearS();
//    //    }
//    //}
//    //tri::QHelper::TDp() = nullptr;//Not sure what this is here for?
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//    //tri::Clean<CMeshO>::RemoveFaceOutOfRangeArea(mesh, 0);
//    //if (nullFaces) log("PostSimplification Cleaning: Removed %d null faces", nullFaces);
//    //int deldupvert = tri::Clean<CMeshO>::RemoveDuplicateVertex(m.cm);
//    //if (deldupvert) log("PostSimplification Cleaning: Removed %d duplicated vertices", deldupvert);
//    int delvert = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
//    //if (delvert) log("PostSimplification Cleaning: Removed %d unreferenced vertices", delvert);
//    //m.clearDataMask(MeshModel::MM_FACEFACETOPO);
//    tri::UpdateTopology<CMeshO>::FaceFace(mesh);
//    tri::Allocator<CMeshO>::CompactVertexVector(mesh);
//    tri::Allocator<CMeshO>::CompactFaceVector(mesh);
//
//    updateBoxAndNormals(mesh);
//    tri::UpdateNormal<CMeshO>::NormalizePerFace(mesh);
//    tri::UpdateNormal<CMeshO>::PerVertexFromCurrentFaceNormal(mesh);
//    tri::UpdateNormal<CMeshO>::NormalizePerVertex(mesh);
//}