#ifndef CLEANUP_H
#define CLEANUP_H
class CMeshO;

namespace Cleanup
{
    void initialCleanup(CMeshO& mesh);
    void ambientOcclusionRemoval(CMeshO& mesh);
    void fixNonManifold(CMeshO& mesh);
    void deleteSmallDisconnectedComponent(CMeshO& mesh);
    void updateBoxAndNormals(CMeshO& mesh);
    int deleteSelectedVerts(CMeshO& mesh);
    
    //bool selected acts only on the selected mesh.
    //bool iterate will iteratively close holes, delete folded faces and delete border vertices to fix difficult holes. 
    void closeHoles(CMeshO& mesh, int maxHoleSize = 500, bool selected = false, bool avoidSelfIntersection = true, bool iterate = false);
    //void closeHolesIteratively()
    void selectBorder(CMeshO& mesh);
    void selectFoldedFaces(CMeshO& mesh);
}

#endif // CLEANUP_H
