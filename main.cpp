#include <QCoreApplication>
#include <QCommandLineParser>
#include <QFileInfo>
#include <QDir>
#include <QDebug>

#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

#include <iostream>

#include "mesh.h"
#include "cleanup.h"

using namespace std;
using namespace vcg;

//NExt step: Save the mesh at the end.



//Figure out why closeholes didn't work?


//After that: implement the ambient occlusion removal.
//After that, replace the QT dependecies with Boost.
//Adapt it so that it can take a list.
//See if I can wrap it for python?

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QCoreApplication::setApplicationName("cleanuptry2");
    QCoreApplication::setApplicationVersion("0.1");

    //Parse command line arguments
    //This will be moved to boost for better portability.
    QCommandLineParser parser;
    parser.setApplicationDescription("Make meshes watertight and 2-manifold. Optionally removes internal meshes through ambient occlusion");
    parser.addHelpOption();

    QCommandLineOption sourceOption(QStringList() << "source" << "s",
                                    "Source file.");
    sourceOption.setDefaultValue("../CleanupTry2/tiny.obj");

    QCommandLineOption destinationOption(QStringList() << "destination" << "d",
                                         "Destination directory.");
    destinationOption.setDefaultValue("output");

    QCommandLineOption ambientOcclusionOption(QStringList() << "ambient_occlusion" << "a",
                                              "Remove internal meshes through ambient occlusion.");

    parser.addOption(sourceOption);
    parser.addOption(destinationOption);
    parser.addOption(ambientOcclusionOption);

    parser.process(a);

    //Assure that the destination directory exists.
    QFileInfo inputFileInfo(parser.value(sourceOption));
    QFileInfo destination;
    destination.setFile(QDir(parser.value(destinationOption)), inputFileInfo.fileName());
    qDebug() << destination.path();
    if(!QFileInfo::exists(destination.path())){
        destination.dir().mkdir(destination.dir().path());
    }

    //Load file
    if(!inputFileInfo.exists()) {
        cout << "File does not exist!" << endl;
    }

    CMeshO mesh;
    mesh.face.EnableNormal();
    mesh.vert.EnableNormal();

    int loadmask = 0;
    if (!vcg::tri::io::Importer<CMeshO>::LoadMask(inputFileInfo.filePath().toLocal8Bit().constData(), loadmask)){
        cout << "Error while loading mask." << endl;
    }
    vcg::tri::io::Importer<CMeshO>::Open(mesh, inputFileInfo.filePath().toLocal8Bit().constData(), loadmask);
    printf("Mesh has %i vertices and %i triangular faces\n",mesh.VN(),mesh.FN());

    //Enable CMeshO modules and update.
    mesh.face.EnableFFAdjacency();
    mesh.vert.EnableVFAdjacency();
    mesh.face.EnableVFAdjacency();

    vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
    vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromFF(mesh);

    mesh.vert.EnableMark();

    //Process Mesh
    Cleanup::initialCleanup(mesh);
    //if it is already 2-manifold, close holes and then run fix to clean up any problems introduced.
//    if
    Cleanup::fixNonManifold(mesh);
    Cleanup::closeHoles(mesh);


    //Save Mesh
    std::string outputFilePath = (destination.path() + "/" + inputFileInfo.fileName()).toLocal8Bit().constData();
    cout << "saving output to " << outputFilePath << endl;
    vcg::tri::io::Exporter<CMeshO>::Save(mesh, outputFilePath.c_str());
    cout << "Finished!";
    return a.exec();
}
