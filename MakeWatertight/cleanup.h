#ifndef CLEANUP_H
#define CLEANUP_H
class CMeshO;

namespace Cleanup
{
    void initialCleanup(CMeshO& mesh);
    void ambientOcclusionRemoval(CMeshO& mesh);
    void fixNonManifold(CMeshO& mesh);
    void updateBoxAndNormals(CMeshO& mesh);
    int deleteSelectedVerts(CMeshO& mesh);
    
    void closeHoles(CMeshO& mesh, int maxHoleSize = 300, bool selected = false, bool avoidSelfIntersection = false);
}

#endif // CLEANUP_H
