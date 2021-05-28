#include "cleanup.h"
#include "mesh.h"
#include "complex.h"

//#include <apps/QT/trimesh_QT_shared/mesh.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/hole.h>

#include "mesh.h"

using namespace vcg;
using namespace std;

void Cleanup::initialCleanup(CMeshO& mesh){
    int result = 0;
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

void Cleanup::ambientOcclusionRemoval(CMeshO& mesh){
//    int result;
//    tri::Allocator<CMeshO>::CompactEveryVector(mesh);
//    tri::UpdateSelection<CMeshO>::VertexClear(mesh);//I don't think I need to do faces?
    //This is going to be a little complicated to implement, as I'll have to use meshlab plugin code rather than vcglib.
    //https://github.com/cnr-isti-vclab/meshlab/blob/f33ff16ca1d78d1bcf46f838dba0b264aa45887d/src/meshlabplugins/filter_ao/filter_ao.cpp
    cout << "ambinetOcclusionRemoval not yet implemented.";
}




//I have excessive updates in here. Once I get things working, I will remove them to make it run faster.
void Cleanup::fixNonManifold(CMeshO& mesh){
    int iteration = 0;
    ///assertion fails below
    /// Assertion failed!

//File: ../../src/vcglib_dev/vcg/simplex/face/pos.h, Line 203

//Expression: nf->V(nf->Prev(nz))!=v && (nf->V(nf->Next(nz))==v || nf->V(nz)==v)



    //First, update
    vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
    tri::UpdateSelection<CMeshO>::VertexClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    tri::Allocator<CMeshO>::CompactVertexVector(mesh);
    tri::Allocator<CMeshO>::CompactFaceVector(mesh);

    if((tri::Clean<CMeshO>::CountNonManifoldVertexFF(mesh) == 0) && (tri::Clean<CMeshO>::CountNonManifoldEdgeFF(mesh) == 0))
    {
        cout << "Mesh is 2-manifold";
        return;
    }

    while(true)
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
        cout << endl <<"Beginning iteration " << iteration << " of fixNonManifold" << endl;
        printf("Mesh has %i vertices and %i triangular faces\n",mesh.VN(),mesh.FN());



        //No pre condition. Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE (Not sure I have to do anything when this is used.)
        result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
        cout << result << " unreferenced vertices removed." << endl;
        printf("Mesh has %i vertices and %i triangular faces\n",mesh.VN(),mesh.FN());
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


        cout  << "Repairing non-manifold edges through vertex splitting." << endl;
        printf("Mesh has %i vertices and %i triangular faces after update.\n",mesh.VN(),mesh.FN());
        //requirements MeshModel::MM_FACEFACETOPO | MeshModel::MM_VERTMARK;
        //Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE
        result = tri::Clean<CMeshO>::SplitManifoldComponents(mesh/*, .005*/);//I need to add a move threshold
        cout << result << " vertices split by SplitManifoldComponents." << endl;
        //This results in a crazy number of vertices! Is this a bug?
        printf("Mesh has %i vertices and %i triangular faces post vertex splitting. \n",mesh.VN(),mesh.FN());



        //No pre condition. Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE (Not sure I have to do anything when this is used.)
        result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
        cout << result << " unreferenced vertices removed." << endl;
        printf("Mesh has %i vertices and %i triangular faces\n",mesh.VN(),mesh.FN());
        if (result != 0) updateBoxAndNormals(mesh);



        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        tri::Allocator<CMeshO>::CompactVertexVector(mesh);
        tri::Allocator<CMeshO>::CompactFaceVector(mesh);









        cout << "Repairing non-manifold vertices by vertex splitting." << endl;
        result = tri::Clean<CMeshO>::SplitNonManifoldVertex(mesh, .005);
        cout << result << " vertices split by SplitManifoldVertex." << endl;
        printf("Mesh has %i vertices and %i triangular faces post vertex splitting. \n",mesh.VN(),mesh.FN());



        //No pre condition. Post condition MM_GEOMETRY_AND_TOPOLOGY_CHANGE (Not sure I have to do anything when this is used.)
        result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
        cout << result << " unreferenced vertices removed." << endl;
        printf("Mesh has %i vertices and %i triangular faces\n",mesh.VN(),mesh.FN());
        if (result != 0) updateBoxAndNormals(mesh);



//        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
//        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
//        tri::Allocator<CMeshO>::CompactVertexVector(mesh);
//        tri::Allocator<CMeshO>::CompactFaceVector(mesh);
//        vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromFF(mesh);





        //select and delete remaining
        tri::UpdateSelection<CMeshO>::VertexClear(mesh);
        int nonManifoldVerts = tri::Clean<CMeshO>::CountNonManifoldVertexFF(mesh);
        if (nonManifoldVerts > 0)
        {
            cout << "Verts, faces prior to deletion:" << mesh.VN() << "," << mesh.FN() << endl;
            int result = deleteSelectedVerts(mesh);
            cout << result << "Non manifold vertices deleted." << endl;
            cout << "Verts, faces post deletion:" << mesh.VN() << "," << mesh.FN() << endl;
        }

        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        tri::Allocator<CMeshO>::CompactVertexVector(mesh);
        tri::Allocator<CMeshO>::CompactFaceVector(mesh);





//        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
//        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
//        vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromFF(mesh);
        Cleanup::updateBoxAndNormals(mesh);//Fails in this call of update.. this is after SplitNonManifoldVertex



        //maybe failure has to do with selecting verts.
        //Now I need to check for non-manifold
        tri::UpdateSelection<CMeshO>::VertexClear(mesh);
        tri::UpdateSelection<CMeshO>::FaceClear(mesh);

        int nonmanivert = tri::Clean<CMeshO>::CountNonManifoldVertexFF(mesh, false, true);
        int nonmaniedge = tri::Clean<CMeshO>::CountNonManifoldEdgeFF(mesh);
        cout << nonmaniedge << " non-manifold edges and " << nonmanivert << " non-manifold vertices." << endl;
        if((nonmanivert == 0) && (nonmaniedge == 0))
        {
            cout << "Mesh is now 2-manifold" << endl;;
            //Close holes and update
            tri::UpdateSelection<CMeshO>::VertexClear(mesh);
            tri::UpdateSelection<CMeshO>::FaceClear(mesh);
            closeHoles(mesh);
            tri::UpdateSelection<CMeshO>::VertexClear(mesh);
            result = tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
            cout << result << " unreferenced vertices removed." << endl;
            vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
            vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
            vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromFF(mesh);
            tri::Allocator<CMeshO>::CompactVertexVector(mesh);
            tri::Allocator<CMeshO>::CompactFaceVector(mesh);

            //Check for 2-manifold again.
            tri::UpdateSelection<CMeshO>::VertexClear(mesh);
            tri::UpdateSelection<CMeshO>::FaceClear(mesh);
            if((tri::Clean<CMeshO>::CountNonManifoldVertexFF(mesh, false, true)) == 0 && (tri::Clean<CMeshO>::CountNonManifoldEdgeFF(mesh) == 0))
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
}

//This method is meant to immitate what meshlab does, called after some methods. Defined in meshmodel.cpp
void Cleanup::updateBoxAndNormals(CMeshO& mesh){
    tri::UpdateBounding<CMeshO>::Box(mesh);

//meshlab uses mesh.fn instead of mesh.FN(), though it tells you never to use fn
    if(mesh.FN()>0) {
        mesh.vert.EnableNormal();
        tri::UpdateNormal<CMeshO>::PerFaceNormalized(mesh);
        tri::UpdateNormal<CMeshO>::PerVertexAngleWeighted(mesh);//Fails here? (*f).V(0)->N() += t*AngleN(e0,-e2); maybe unreferenced face? NormalType &N()       { assert((*this).Base().NormalEnabled); return (*this).Base().NV[(*this).Index()];  }
    }
}








int Cleanup::deleteSelectedVerts(CMeshO& mesh){
    //Flag as deleted all vertices that are flagged as selected.
    //Might be more effecient for me to re-write CountNonManifoldVertexFF and flag as delete instead of selected.
    //meshlab does this chunked?
    CMeshO::FaceIterator faceIterator;
    CMeshO::VertexIterator vertexIterator;
    int deleted = 0;

    tri::UpdateSelection<CMeshO>::FaceClear(mesh);
    tri::UpdateSelection<CMeshO>::FaceFromVertexLoose(mesh);


    int vvn = mesh.vn;
    int ffn = mesh.fn;
    for (faceIterator = mesh.face.begin(); faceIterator != mesh.face.end(); ++faceIterator)
        if (!(*faceIterator).IsD() && (*faceIterator).IsS())
            tri::Allocator<CMeshO>::DeleteFace(mesh, *faceIterator);
    for (vertexIterator = mesh.vert.begin(); vertexIterator != mesh.vert.end(); ++vertexIterator)
        if (!(*vertexIterator).IsD() && (*vertexIterator).IsS())
        {
            tri::Allocator<CMeshO>::DeleteVertex(mesh, *vertexIterator);
            deleted++;
        }
//    m.clearDataMask(MeshModel::MM_FACEFACETOPO);
//    m.clearDataMask(MeshModel::MM_VERTFACETOPO);
    updateBoxAndNormals(mesh);
    printf("Deleted %i vertices, %i faces.", vvn - mesh.vn, ffn - mesh.fn);









//    int deleted = 0;
//    for (CVertexO& vertex : mesh.vert)
//            {
//                if (!vertex.IsD() && vertex.IsS())
//                {
//                    tri::Allocator<CMeshO>::DeleteVertex(mesh, vertex);//as far as I know, this should also handle associated faces
//                    ++deleted;
//                }
//            }
    return deleted;
}







void Cleanup::closeHoles(CMeshO& mesh, int maxHoleSize, bool selected , bool avoidSelfIntersection){
    cout << "Closing holes." << endl;
    if(avoidSelfIntersection)
        tri::Hole<CMeshO>::EarCuttingIntersectionFill<tri::SelfIntersectionEar< CMeshO> >(mesh,maxHoleSize,selected);
    else
        tri::Hole<CMeshO>::EarCuttingFill<vcg::tri::MinimumWeightEar< CMeshO> >(mesh,maxHoleSize,selected);
}
