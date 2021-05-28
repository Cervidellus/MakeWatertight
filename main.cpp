#include <QCoreApplication>
#include <QCommandLineParser>
#include <QFileInfo>
#include <QDebug>

#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>

#include <iostream>

#include "mesh.h"
#include "cleanup.h"

using namespace std;
using namespace vcg;

//NExt step: convert fix_non_manifold to while loop.
//After that: implement the ambient occlusion removal.

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QCoreApplication::setApplicationName("cleanuptry2");
    QCoreApplication::setApplicationVersion("0.1");

    //Parse command line arguments
    QCommandLineParser parser;
    parser.setApplicationDescription("Make meshes watertight and 2-manifold. Optionally removes internal meshes through ambient occlusion");
    parser.addHelpOption();

    QCommandLineOption sourceOption(QStringList() << "source" << "s",
                                    "Source file.");
    sourceOption.setDefaultValue("../CleanupTry2/864691136023902009.obj");

    QCommandLineOption destinationOption(QStringList() << "destination" << "d",
                                         "Destination directory.");
    destinationOption.setDefaultValue("output");

    QCommandLineOption ambientOcclusionOption(QStringList() << "ambient_occlusion" << "a",
                                              "Remove internal meshes through ambient occlusion.");

    parser.addOption(sourceOption);
    parser.addOption(destinationOption);
    parser.addOption(ambientOcclusionOption);

    parser.process(a);

    QString qfileName = parser.value(sourceOption);

    //Load file
    if(!QFile::exists(qfileName)) {
        cout << "File does not exist!" << endl;
    }
    int loadmask = 0;
    string filename = QFile::encodeName(qfileName).constData();

    CMeshO mesh;
    mesh.face.EnableNormal();
    mesh.vert.EnableNormal();
    if (!vcg::tri::io::Importer<CMeshO>::LoadMask(filename.c_str(), loadmask)){
        cout << "Error while loading mask.";
    }
    vcg::tri::io::Importer<CMeshO>::Open(mesh, filename.c_str(), loadmask);
    printf("Mesh has %i vertices and %i triangular faces\n",mesh.VN(),mesh.FN());

//    //Enable components
//    vcg::tri::Allocator<CMeshO>::CompactFaceVector(mesh);//Not sure if needed
//    vcg::tri::Allocator<CMeshO>::CompactVertexVector(mesh);//Not sure if needed

    mesh.face.EnableFFAdjacency();
    mesh.vert.EnableVFAdjacency();
    mesh.face.EnableVFAdjacency();


    vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
    vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromFF(mesh);

//    mesh.face.EnableNormal();
//    mesh.vert.EnableNormal();
//    Cleanup::updateBoxAndNormals(mesh);
    mesh.vert.EnableMark();

//    mesh.vert.EnableQuality();

    Cleanup::initialCleanup(mesh);
    Cleanup::fixNonManifold(mesh);

    cout << "Finished!";



    return a.exec();
}
