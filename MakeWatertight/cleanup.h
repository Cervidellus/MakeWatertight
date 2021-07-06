#ifndef CLEANUP_H
#define CLEANUP_H
class CMeshO;

namespace Cleanup
{
    void makeManifoldAndWatertight(CMeshO& mesh, bool ambientOcclusion = true);
    void initialCleanup(CMeshO& mesh);
    void ambientOcclusionRemoval(CMeshO& mesh);
    bool fixNonManifold(CMeshO& mesh);
    void deleteSmallDisconnectedComponent(CMeshO& mesh);
    void updateBoxAndNormals(CMeshO& mesh);
    int deleteSelectedFacesAndVerts(CMeshO& mesh);

    int deleteSelectedFaces(CMeshO& mesh);
    int deleteSelectedVertices(CMeshO& mesh);
    
    //bool selected acts only on the selected mesh.
    //If there are remaining border faces, it returns false with border verts and faces selected
    bool closeHoles(CMeshO& mesh, int maxHoleSize = 500, bool selected = false, bool avoidSelfIntersection = true);

    void selectBorder(CMeshO& mesh);
    void selectFoldedFaces(CMeshO& mesh);
}

#endif // CLEANUP_H
