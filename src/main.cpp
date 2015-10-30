#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/gp3.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <RGBDReader/RGBDReader.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;


int main(int argc, char *argv[]) {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

    // Parse arguments
    po::options_description desc;
    desc.add_options()
        ("help", "Show help message")
        ("show", "Show the tessellation")
        ("input,i", po::value<std::string>()->required(), "Point cloud input file (required)")
        ("output,o", po::value<std::string>(), "Output model file");
    po::variables_map vm;

    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (po::required_option& e) {
        std::cout << desc;
        return -1;
    }

    fs::path input_file = vm["input"].as <std::string>();
    fs::path output_file;

    pcl::PolygonMesh triangles;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    RGBDReader::RGBD_TUM_Reader::Ptr tum_reader(new RGBDReader::RGBD_TUM_Reader());

    if (input_file.extension() == ".png") {

        // Read the cloud from a image of the TUM dataset.
        tum_reader->readCloud(input_file.string(), *cloud);

        // Tessellation for organized clouds
        pcl::OrganizedFastMesh<pcl::PointXYZ>::Ptr fastmesh(new pcl::OrganizedFastMesh<pcl::PointXYZ>());
        fastmesh->setInputCloud(cloud);
        fastmesh->setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_RIGHT_CUT);
        fastmesh->reconstruct(triangles);

    } else if (input_file.extension() == ".pcd") {

        // Normal unorganized cloud
        pcl::io::loadPCDFile<pcl::PointXYZ> (input_file.string(), *cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(10);
        ne.compute(*normals);

        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius (0.05);

        // Set typical values for the parameters
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        gp3.setMinimumAngle(M_PI/20); // 10 degrees
        gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
        gp3.setNormalConsistency(true);
        gp3.setConsistentVertexOrdering(true);

        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);
    }

    if (vm.count("output")) {
        output_file = vm["output"].as <std::string>();
    } else {
        output_file = fs::current_path();
        output_file /= input_file.filename();
        output_file.replace_extension(".ply");
    }

    if (!output_file.has_extension()) {
        output_file.replace_extension(".ply");
    } else {
        if (fs::extension(output_file).compare(".vtk") == 0) {
            pcl::io::saveVTKFile (output_file.string(), triangles);
        }
        else if (fs::extension(output_file).compare(".ply") == 0) {
            pcl::io::savePLYFile (output_file.string(), triangles);
        } else {
            std::cerr << "Unknown output format" << std::endl;
            return 1;
        }
    }

    if (vm.count("show")) {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
        viewer->addPolygonMesh(triangles);

        // Display normals
        //viewer->addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>(cloud_with_normals, cloud_with_normals, 1, 0.05, "normals");

        while (!viewer->wasStopped()) {
            viewer->spinOnce (100);
        }
    }

    return 0;
}
