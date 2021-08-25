//vcglib uses some pre-c++11 code practices that are now considered either unsafe or deprecated. 
//To compile correctly with MSVC, the /permissive option must be used.
//In visual studio, find it under c/c++::language::Conformance mode
//_CRT_SECURE_NO_WARNINGS must also be defined as a preprocessor command. 

#include <iostream>
#include <chrono>

#include "boost/program_options.hpp"
#include "boost/filesystem.hpp"
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

#include "mesh.h"
#include "cleanup.h"

using namespace std;
namespace po = boost::program_options;

int main(int argc, char* argv[])
{
    //Process command line arguments
    po::options_description description("Takes a trimesh as input and outputs a watertight and 2-manifold mesh.");

    description.add_options()
        ("help, h", "Display this help message.")

        ("source, s",
            po::value<string>()->default_value("864691134884871930.ply"),
            //po::value<string>()->default_value("internal_test.ply"),
            "Source file.")
        
        ("destination, d",
            po::value<string>()->default_value("output"),
            "Destination folder. Will create if does not exist.")

        ("ambient_occlusion, a",
            po::value<bool>()->default_value(true), 
            "Deletion of internal geometries based on screen space ambient occlusion values. Default True.")

        ("remove_folded, f", 
            po::value<bool>()->default_value(false),
            "Delete folded faces if mesh not watertight.")

        ("decimation_ratio, d",
            po::value<double>()->default_value(0.0),
            "If greater than 0, decimate mesh after watertight. Double represents the ratio of the finished mesh size to the initial mesh size."
            )
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
    auto startTime = std::chrono::high_resolution_clock::now();
    Cleanup::makeManifoldAndWatertight(mesh, vm["ambient_occlusion"].as<bool>(), vm["remove_folded"].as<bool>());
    //double decimation_ratio = vm["decimation_ratio"].as<double>();
    //if (decimation_ratio > 0) {
    //    Cleanup::decimate(mesh, decimation_ratio);
    //}
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> runTime = endTime - startTime;

    std::cout << runTime.count() << " seconds to compute makeManifoldAndWatertight." << endl;

    //Save Mesh
    std::string outputFilePath = (destination.string() + "/" + source.filename().string());
    cout << "saving output to " << outputFilePath << endl;
    vcg::tri::io::Exporter<CMeshO>::Save(mesh, outputFilePath.c_str(), vcg::tri::io::Mask::IOM_VERTCOLOR | vcg::tri::io::Mask::IOM_VERTQUALITY);

    cout << "Finished!";
    return 0;
}

//Next steps:



//Write to process a list instead of a string. 

//Try and figure out how to get vertex splitting to take less memory.

//Figure out why vertex splitting often reports 1 vertex. 

//SplitNonManifold creates HUGE numbers of points, which causes memory issues... 

//Add a decimation option.. postpone for now.. it is heavily wrapped in QT, and for now I will just use python. 


//Biggest ways to speed things up:
//don't get rid of folded faces.
//move ambient occlusion to gpu.
//start with better meshes to begin with... complicated non-manifold geometries require multiple iterations. T-vertices can cause issues. 
//Multithreading would allow it to run over a list much faster, especially if combined with moving the Ambient Occlusion to GPU. 

//Figuring out exactly why it cannot close holes might lead to clues on how to greatly speed it up. SplitManifoldComponents is slow, and is often just dealing with couple of vertices. 
//It might be best just to do it if the number of non-manifold verts is greater than a certain number. 

//Interesting:
//https://stackoverflow.com/questions/52160757/cgal-hole-filling-with-color