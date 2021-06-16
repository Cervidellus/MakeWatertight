//vcglib uses some pre-c++11 code practices that are now considered either unsafe or deprecated. 
//To compile correctly with MSVC, the /permissive option must be used.
//In visual studio, find it under c/c++::language::Conformance mode
//_CRT_SECURE_NO_WARNINGS must also be defined as a preprocessor command. 

#include "boost/program_options.hpp"
#include "boost/filesystem.hpp"
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

#include "mesh.h"
#include "cleanup.h"

#include <iostream>
using namespace std;
namespace po = boost::program_options;

int main(int argc, char* argv[])
{
    //Process command line arguments
    po::options_description description("Takes a trimesh as input and outputs a watertight and 2-manifold mesh.");

    description.add_options()
        ("help, h", "Display this help message.")

        ("source, s",
            po::value<string>()->default_value("864691136023902009.ply"),
            //po::value<string>()->default_value("internal_test.ply"),
            "Source file.")
        
        ("destination, d",
            po::value<string>()->default_value("output"),
            "Destination folder. Will create if does not exist.")

        ("ambient_occlusion, a",
            po::value<bool>()->default_value(true), 
            "Deletion of internal geometries based on screen space ambient occlusion values. Default True.")
        ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(), vm);
    po::notify(vm);

    boost::filesystem::path source{vm["source"].as<string>()};
    boost::filesystem::path destination{vm["destination"].as<string>()};

    if(!boost::filesystem::exists(destination))
    {
        cout << "Creating destination directory:" << destination.string() <<  endl;
        boost::filesystem::create_directory(vm["destination"].as<string>());
    }

    CMeshO mesh;
    mesh.face.EnableNormal();
    mesh.vert.EnableNormal();

    //Load file

    if(!boost::filesystem::exists(source.string())) {
        cout << "Source file does:" << source.root_path() << " does not exist!" << endl;
        cout << "current path: " << boost::filesystem::current_path() << endl;
        return 1;
    }

    int loadmask = 0;
    if (!vcg::tri::io::Importer<CMeshO>::LoadMask(source.string().data(), loadmask)){
        cout << "Error while loading mask." << endl;
    }
    vcg::tri::io::Importer<CMeshO>::Open(mesh, source.string().data(), loadmask);
    printf("Mesh has %i vertices and %i triangular faces\n",mesh.VN(),mesh.FN());

        //Enable CMeshO modules and update.
        mesh.face.EnableFFAdjacency();
        mesh.vert.EnableVFAdjacency();
        mesh.face.EnableVFAdjacency();

        vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
        vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);
        vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromFF(mesh);

        mesh.vert.EnableMark();
        mesh.vert.EnableQuality();
        mesh.vert.EnableCurvature();
        mesh.vert.EnableCurvatureDir();
        mesh.vert.EnableRadius();
        mesh.vert.EnableColor();

        //Process Mesh
        Cleanup::makeManifoldAndWatertight(mesh, vm["ambient_occlusion"].as<bool>());

        //Save Mesh
        std::string outputFilePath = (destination.string() + "/" + source.filename().string());
        cout << "saving output to " << outputFilePath << endl;
        vcg::tri::io::Exporter<CMeshO>::Save(mesh, outputFilePath.c_str(), vcg::tri::io::Mask::IOM_VERTCOLOR | vcg::tri::io::Mask::IOM_VERTQUALITY);

        cout << "Finished!";
        return 0;
}

//Next steps:

//Iteratively delete border if close holes fails at end of fixNonManifold. Then check manifold again.
    //Figure out where it needs updating, and get rid of some of hte extra bits...

//Write to process a list instead of a string. 

//Try and figure out how to get vertex splitting to take less memory.