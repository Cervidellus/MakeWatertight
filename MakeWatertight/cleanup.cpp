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

#include <meshlab/src/meshlabplugins/filter_mls/smallcomponentselection.h>

#include <igl/embree/ambient_occlusion.h>

typedef unsigned int uint;
using namespace vcg;
using namespace std;

void Cleanup::initialCleanup(CMeshO& mesh) {
    int result = 0;
    deleteSmallDisconnectedComponent(mesh);
    cout << " Deleted small disconnected component." << endl;
    //No pre condition. Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE (Not sure I have to do anything when this is used.)
    result = tri::Clean<CMeshO>::RemoveFaceOutOfRangeArea(mesh, 0);//This I believe causes some unfortunate holes...and susequent t-vertices, that I need to fix.
    cout << result << " Zero area vertices removed." << endl;
    //after this they call clearDataMask(MM_FACEFACETOPO). This in turn calls cm.face.DisableFFAdjacency(); Not sure I want to do that?
    //I think what it does is enable this in the method, and then we can clear if we don't need?

    //No pre condition. Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE (Not sure I have to do anything when this is used.)
    result = tri::Clean<CMeshO>::RemoveDuplicateFace(mesh);
    cout << result << " Duplicate Faces removed." << endl;

    //No pre condition. Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE (Not sure I have to do anything when this is used.)
    result = tri::Clean<CMeshO>::RemoveDuplicateVertex(mesh);
    cout << result << " duplicate vertices removed." << endl;
    if (result != 0) updateBoxAndNormals(mesh);
    //meshlab clears MM_FACEFACETOPO and MM_VERTFACETOPO. I don't want to do that?

//    result = tri::Clean<CMeshO>::MergeCloseVertex(mesh, .001);
//    cout << result << " close vertices merged." << endl; //this creates some non-manifold and is slow

    //No pre condition. Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE (Not sure I have to do anything when this is used.)
    result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
    cout << result << " unreferenced vertices removed." << endl;
    if (result != 0) updateBoxAndNormals(mesh);
}

void Cleanup::ambientOcclusionRemoval(CMeshO& mesh) {
    //https://github.com/cnr-isti-vclab/meshlab/blob/585226d8a28244379c2807b6848634c4c545c4b7/src/meshlabplugins/filter_ao/filter_ao.cpp
    //This might be our best bet: https://github.com/libigl/libigl/blob/main/tutorial/606_AmbientOcclusion/main.cpp
    //meshlab is actually using it underneath for some things, so I should be able to figure out type conversion.
    //https://github.com/cnr-isti-vclab/vcglib/blob/001a01b38688acbf81f0ec821e9bde3e6d4622ab/wrap/igl/smooth_field.h
    //They use a MeshToMatrix function to get the appropriate vertex and face matrices.

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
    vcg::tri::MeshToMatrix<CMeshO>::GetTriMeshData(mesh,F,V);
    vcg::tri::MeshToMatrix<CMeshO>::GetNormalData(mesh, NV, NF);

    //Step two: Use libigl to calculate the ambient occlusion.
        //I get some errors when I include <igl/ambient_occlusion.h>
    //    igl::embree::ambient_occlusion(V,F,V,NV,2500,AO);
    cout << "Calculating ambient occlusion." << endl;
    igl::embree::ambient_occlusion(V,F,V,NV,500,AO);
    cout << "Applying ambient occlusion." << endl;

    for (vi = mesh.vert.begin(); vi != mesh.vert.end(); ++vi)
        if (!(*vi).IsD())
        {
            //cout << vi->Index() << " ";     
            vi->Q() = AO[vi->Index()];
            //cout << vi->Q() << endl;
        }
     //This here temporarily to check
    //vcg::tri::UpdateColor<CMeshO>::PerVertexQualityGray(mesh);
    //vcg::tri::UpdateColor<CMeshO>::PerVertexConstant(mesh, vcg::Color4b::Red);

    //Step four: Delete faces and vertices based on that quality.
    //Modeled after meshlab FP_SELECT_DELETE_FACEVERT, with the exception that I select based on quality 

    tri::UpdateSelection<CMeshO>::VertexFromQualityRange(mesh, .95, 1);
    //tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceFromVertexStrict(mesh);
    //int vvn = mesh.vn;
    //int ffn = mesh.fn;

    //deleteSelectedVerts(mesh);
    vcg::tri::SmallComponent<CMeshO>::DeleteFaceVert(mesh);
    std::printf("Mesh has %i vertices and %i triangular faces after ambient occlusion removal.\n", mesh.VN(), mesh.FN());


    //for (fi = mesh.face.begin(); fi != mesh.face.end(); ++fi)
    //    if (!(*fi).IsD() && (*fi).IsS())
    //        tri::Allocator<CMeshO>::DeleteFace(mesh, *fi);

    ////Is something wrong here? Seems like a bunch of verts go away... 
    //for (vi = mesh.vert.begin(); vi != mesh.vert.end(); ++vi)
    //    if (!(*vi).IsD() && (*vi).IsS())
    //        tri::Allocator<CMeshO>::DeleteVertex(mesh, *vi);

    tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    //int result = tri::Clean<CMeshO>::RemoveFaceOutOfRangeArea(mesh, 0);//This I believe causes some unfortunate holes...and susequent t-vertices, that I need to fix.
    //cout << result << " Zero area vertices removed." << endl;
    tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);

    vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
    tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    tri::Allocator<CMeshO>::CompactVertexVector(mesh);
    std::printf("Mesh has %i vertices and %i triangular faces after cleanup.\n", mesh.VN(), mesh.FN());//why is this low?
    tri::Allocator<CMeshO>::CompactFaceVector(mesh);

    deleteSmallDisconnectedComponent(mesh);
    //updateBoxAndNormals(mesh);
}

void Cleanup::fixNonManifold(CMeshO& mesh) {
    int iteration = 0;

    vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
    tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    tri::Allocator<CMeshO>::CompactVertexVector(mesh);
    tri::Allocator<CMeshO>::CompactFaceVector(mesh);

    if ((tri::Clean<CMeshO>::CountNonManifoldVertexFF(mesh) == 0) && (tri::Clean<CMeshO>::CountNonManifoldEdgeFF(mesh) == 0))
    {
        cout << "Mesh is 2-manifold at start of fixNonManifold. \n Closing holes." << endl;
        closeHoles(mesh);
        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        if ((tri::Clean<CMeshO>::CountNonManifoldVertexFF(mesh) == 0) && (tri::Clean<CMeshO>::CountNonManifoldEdgeFF(mesh) == 0))
        {
            cout << "Mesh remains 2-manifold. Skipping fixNonManifold iteration." << endl;
            return;
        }
        else
        {
            cout << "Cleanup::closeHoles() introduced non-manifold geometry. Proceeding with fixNonManifold iteration." << endl;
        }
    }

    while (true)
    {
        iteration++;
        int result = 0;
        tri::UpdateSelection<CMeshO>::VertexClear(mesh);
        tri::UpdateSelection<CMeshO>::FaceClear(mesh);
        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        tri::Allocator<CMeshO>::CompactVertexVector(mesh);
        tri::Allocator<CMeshO>::CompactFaceVector(mesh);
        updateBoxAndNormals(mesh);
        cout << endl << "Beginning iteration " << iteration << " of fixNonManifold" << endl;
        std::printf("Mesh has %i vertices and %i triangular faces\n", mesh.VN(), mesh.FN());

        //No pre condition. Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE (Not sure I have to do anything when this is used.)
        result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
        cout << result << " unreferenced vertices removed." << endl;
        std::printf("Mesh has %i vertices and %i triangular faces\n", mesh.VN(), mesh.FN());
        if (result != 0) updateBoxAndNormals(mesh);

        //requirements MeshModel::MM_FACEFACETOPO | MeshModel::MM_VERTMARK;
        cout << "Repairing non-manifold edges through incident face removal." << endl;
        result = tri::Clean<CMeshO>::RemoveNonManifoldFace(mesh);
        cout << result << " faces removed." << endl;
        if (result != 0) updateBoxAndNormals(mesh);

        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        tri::Allocator<CMeshO>::CompactVertexVector(mesh);
        tri::Allocator<CMeshO>::CompactFaceVector(mesh);

        cout << "Repairing non-manifold edges through vertex splitting." << endl;
        std::printf("Mesh has %i vertices and %i triangular faces after update.\n", mesh.VN(), mesh.FN());
        //requirements MeshModel::MM_FACEFACETOPO | MeshModel::MM_VERTMARK;
        //Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE
        result = tri::Clean<CMeshO>::SplitManifoldComponents(mesh/*, .005*/);//I need to add a move threshold
        cout << result << " vertices split by SplitManifoldComponents." << endl;
        //This results in a crazy number of unreferenced vertices! Is this a bug?
        std::printf("Mesh has %i vertices and %i triangular faces post vertex splitting. \n", mesh.VN(), mesh.FN());

        //No pre condition. Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE (Not sure I have to do anything when this is used.)
        result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
        cout << result << " unreferenced vertices removed." << endl;
        std::printf("Mesh has %i vertices and %i triangular faces\n", mesh.VN(), mesh.FN());
        if (result != 0) updateBoxAndNormals(mesh);

        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        tri::Allocator<CMeshO>::CompactVertexVector(mesh);
        tri::Allocator<CMeshO>::CompactFaceVector(mesh);

        //No pre condition. Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE (Not sure I have to do anything when this is used.)
        cout << "Repairing non-manifold vertices by vertex splitting." << endl;
        result = tri::Clean<CMeshO>::SplitNonManifoldVertex(mesh, .005);
        cout << result << " vertices split by SplitManifoldVertex." << endl;
        std::printf("Mesh has %i vertices and %i triangular faces post vertex splitting. \n", mesh.VN(), mesh.FN());
        if (result != 0) updateBoxAndNormals(mesh);

        //No pre condition. Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE (Not sure I have to do anything when this is used.)
        result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
        cout << result << " unreferenced vertices removed." << endl;
        std::printf("Mesh has %i vertices and %i triangular faces\n", mesh.VN(), mesh.FN());
        if (result != 0) updateBoxAndNormals(mesh);

        //select and delete remaining
        tri::UpdateSelection<CMeshO>::VertexClear(mesh);
        int nonManifoldVerts = tri::Clean<CMeshO>::CountNonManifoldVertexFF(mesh);
        if (nonManifoldVerts > 0)
        {
            cout << "Verts, faces prior to deletion:" << mesh.VN() << "," << mesh.FN() << endl;
            //int result = deleteSelectedVerts(mesh);
            tri::SmallComponent<CMeshO>::DeleteFaceVert(mesh);
            //cout << result << "Non manifold vertices deleted." << endl;
            cout << "Verts, faces post deletion:" << mesh.VN() << "," << mesh.FN() << endl;
        }

        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        tri::Allocator<CMeshO>::CompactVertexVector(mesh);
        tri::Allocator<CMeshO>::CompactFaceVector(mesh);

        Cleanup::updateBoxAndNormals(mesh);

        //        tri::UpdateSelection<CMeshO>::VertexClear(mesh);
        //        tri::UpdateSelection<CMeshO>::FaceClear(mesh);

        int nonmanivert = tri::Clean<CMeshO>::CountNonManifoldVertexFF(mesh, false, true);
        int nonmaniedge = tri::Clean<CMeshO>::CountNonManifoldEdgeFF(mesh);
        cout << nonmaniedge << " non-manifold edges and " << nonmanivert << " non-manifold vertices." << endl;
        if ((nonmanivert == 0) && (nonmaniedge == 0))
        {
            cout << "Mesh is now 2-manifold" << endl;

            //exception thrown line 571 of component_ocf
            // vector_ocf<typename T::VertexType> &Base() const { return *_ovp;}
            // this was nullpointer
            //very likely I need to compact things. 
            closeHoles(mesh, 500, false, true, true);
            }

        //Check for 2-manifold again.
        tri::UpdateSelection<CMeshO>::VertexClear(mesh);
        tri::UpdateSelection<CMeshO>::FaceClear(mesh);
        if ((tri::Clean<CMeshO>::CountNonManifoldVertexFF(mesh, false, true)) == 0 && (tri::Clean<CMeshO>::CountNonManifoldEdgeFF(mesh) == 0))
        {
            cout << "Finished fixNonManifold." << endl;

            break;
        }
        else
        {
            cout << "Close-holes introduced non 2-manifold geometry." << endl;
        }
    }
}

//This immitates Meshlab. Defined in meshmodel.cpp
void Cleanup::updateBoxAndNormals(CMeshO& mesh) {
    tri::UpdateBounding<CMeshO>::Box(mesh);

    //meshlab uses mesh.fn instead of mesh.FN(), though it tells you never to use fn
    if (mesh.FN() > 0) {
        mesh.vert.EnableNormal();
        tri::UpdateNormal<CMeshO>::PerFaceNormalized(mesh);
        tri::UpdateNormal<CMeshO>::PerVertexAngleWeighted(mesh);//crash here after AO?
    }
}

int Cleanup::deleteSelectedVerts(CMeshO& mesh) {
    //Flag as deleted all vertices that are flagged as selected.
    //Might be more effecient for me to re-write CountNonManifoldVertexFF and flag as delete instead of selected.
    CMeshO::FaceIterator faceIterator;
    CMeshO::VertexIterator vertexIterator;
    int deleted = 0;
    int vvn = mesh.vn;
    int ffn = mesh.fn;

    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceFromVertexLoose(mesh);

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
    std::printf("Deleted %i vertices, %i faces.", vvn - mesh.vn, ffn - mesh.fn);

    return deleted;
}

void Cleanup::closeHoles(CMeshO& mesh, int maxHoleSize, bool selected, bool avoidSelfIntersection, bool iterate) {
    cout << "Closing holes." << endl;
    if (iterate)
    {
        while (true)
        {
            //Maybe check for non_manifold here? Maybe have it return a success bool, so non-manifold can be handled elsewhere. 
            // Or... maybe take the while loop out of cleanup ( I think this is the better idea...) that way I can handle the manifold check in a nicer way. 
            //first try filling the hole.
            tri::Hole<CMeshO>::EarCuttingIntersectionFill<tri::SelfIntersectionEar< CMeshO> >(mesh, maxHoleSize, selected);
            //If that fails, delete folded faces and remaining border.
            selectBorder(mesh);
            int borderVerts = tri::UpdateSelection<CMeshO>::VertexCount(mesh);
            int borderFaces = tri::UpdateSelection<CMeshO>::FaceCount(mesh);
            cout << borderVerts << " border vertices and " << borderFaces << " found after closing holes." << endl;
            if (borderFaces > 0)
            {
                cout << "Deleting border and folded faces." << endl;
                //select and delete folded faces
                selectFoldedFaces(mesh);//crashes here
                int foldedCount = tri::UpdateSelection<CMeshO>::FaceCount(mesh);
                if (foldedCount > 0)
                {
                    cout << "Deleting " << foldedCount << " folded faces." << endl;
                    tri::SmallComponent<CMeshO>::DeleteFaceVert(mesh);
                }
                selectBorder(mesh);
                tri::SmallComponent<CMeshO>::DeleteFaceVert(mesh);
                deleteSmallDisconnectedComponent(mesh);
                tri::UpdateTopology<CMeshO>::FaceFace(mesh);
                tri::UpdateTopology<CMeshO>::VertexFace(mesh);
                tri::Allocator<CMeshO>::CompactVertexVector(mesh);
                tri::Allocator<CMeshO>::CompactFaceVector(mesh);

                //NEED TO CHECK FOR NON_MANIFOLD BEFORE CLOSING HOLES>>> 
            }
            else break;
        }
    }
    else
    {
        if (!avoidSelfIntersection)
            tri::Hole<CMeshO>::EarCuttingIntersectionFill<tri::SelfIntersectionEar< CMeshO> >(mesh, maxHoleSize, selected);
        else
            tri::Hole<CMeshO>::EarCuttingFill<tri::MinimumWeightEar< CMeshO> >(mesh, maxHoleSize, selected);
    }

}

void Cleanup::selectBorder(CMeshO& mesh)
{
    tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    tri::UpdateFlags<CMeshO>::FaceBorderFromNone(mesh);
    tri::UpdateFlags<CMeshO>::VertexBorderFromFaceBorder(mesh);
    tri::UpdateSelection<CMeshO>::FaceFromBorderFlag(mesh);
    tri::UpdateSelection<CMeshO>::VertexFromBorderFlag(mesh);
}

void Cleanup::selectFoldedFaces(CMeshO& mesh)
{
    tri::Allocator<CMeshO>::CompactVertexVector(mesh);
    tri::Allocator<CMeshO>::CompactFaceVector(mesh);
    tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    tri::UpdateTopology<CMeshO>::VertexFace(mesh);
    tri::Clean<CMeshO>::SelectFoldedFaceFromOneRingFaces(mesh, 2.9);//2.9 is roughly 170 degrees.
}

void Cleanup::deleteSmallDisconnectedComponent(CMeshO& mesh) {
    vcg::tri::SmallComponent<CMeshO>::Select(mesh, 0.1);
    vcg::tri::SmallComponent<CMeshO>::DeleteFaceVert(mesh);
}
