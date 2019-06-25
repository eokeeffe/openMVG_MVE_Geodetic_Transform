#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/ostreamwrapper.h>

#include <boost/filesystem.hpp>

using namespace boost::filesystem;
using namespace rapidjson;

#define DEBUG_ACTIVE 1

// This function displays the help
void
showHelp(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply] [geodectic_transform.json]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}



// This is the main function
int
main (int argc, char** argv)
{
    bool mesh = false;
    bool pointcloud = false;
    int c = 0;
    char *inputfile = NULL;
    char *jsonfile = NULL;

    while ((c = getopt (argc, argv, "hi:f:mp")) != EOF){
        switch(c)
        {
            case 'h':{
                showHelp (argv[0]);
                return 0;
            }
            case 'i':{
                inputfile = optarg;
                break;
            }
            case 'f':{
                jsonfile = optarg;
                break;
            }
            case 'm':{
                mesh = true;
                break;
            }
            case 'p':{
                pointcloud = true;
                break;
            }
            default:{continue;}
        }
    }
    
    if((!mesh) && (!pointcloud)){
        std::cerr << "No m or p options detected" << std::endl;
        return -1;
    }
    
    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    if(mesh){
        pcl::PolygonMesh mesh_binary_pcl;
        
        if (pcl::io::loadPLYFile (inputfile, mesh_binary_pcl) < 0)  {
            std::cout << "Error loading point cloud " << inputfile << std::endl << std::endl;
            showHelp (argv[0]);
            return -1;
        }

        std::stringstream ss;
        std::ifstream file(jsonfile);
        if (file) {
            ss << file.rdbuf();
            file.close();
        } else {
            throw std::runtime_error("!! Unable to open json file");
        }
        
        Document document;
        document.Parse(ss.str().c_str());
        
        // Start parsing json string
        double scale = document["scale"].GetDouble();
        double translations[3];
        double rotations[3][3];
        
        Value& a = document["translation"];
        if(a.IsArray()){
            for (SizeType i = 0; i < a[0].Size(); i++){
                 double tmp = a[0][i].GetDouble();
                 //std::cout<<tmp<<", "<<std::endl;
                 translations[i] = tmp;
            }
        }
        
        Value& b = document["rotation"];
        if(b.IsArray()){
            for(int y=0;y<b.Size();y++){
                for(int i=0;i<b.Size();i++){
                    rotations[y][i] = b[y][i].GetDouble();
                }
            }
        }
        
        if(DEBUG_ACTIVE){
            std::cout << scale << std::endl;
            for(int x=0;x<3;x++){
                std::cout << translations[x] << " , ";
            }
            std::cout << std::endl;
            for(int x=0;x<3;x++){
                for(int y=0;y<3;y++){
                    std::cout << rotations[x][y] << " , ";
                }
                std::cout << std::endl;
            }
        }
        
        //exit(0);

        /* Reminder: how transformation matrices work :
        |-------> This column is the translation
        | 1 0 0 x |  \
        | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
        | 0 0 1 z |  /
        | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
        METHOD #1: Using a Matrix4f
        This is the "manual" method, perfect to understand but error prone !
        */
        Eigen::Matrix4f rotation_translation_m = Eigen::Matrix4f::Identity(),
            scale_m = Eigen::Matrix4f::Identity(),
            final_m = Eigen::Matrix4f::Identity();

        rotation_translation_m (0,0) = rotations[0][0];
        rotation_translation_m (0,1) = rotations[0][1];
        rotation_translation_m (0,2) = rotations[0][2];
        
        rotation_translation_m (1,0) = rotations[1][0];
        rotation_translation_m (1,1) = rotations[1][1];
        rotation_translation_m (1,2) = rotations[1][2];
        
        rotation_translation_m (2,0) = rotations[2][0];
        rotation_translation_m (2,1) = rotations[2][1];
        rotation_translation_m (2,2) = rotations[2][2];
        //    (row, column)

        // Define a translation of 2.5 meters on the x axis.
        rotation_translation_m (0,3) = translations[0];
        rotation_translation_m (1,3) = translations[1];
        rotation_translation_m (2,3) = translations[2];

        scale_m (0,0) = scale;
        scale_m (1,1) = scale;
        scale_m (2,2) = scale;

        // TransformedVector = TranslationMatrix * RotationMatrix * ScaleMatrix * Or0iginalVector;
        // scaling FIRST, and THEN the rotation, and THEN the translation
        final_m = rotation_translation_m * scale_m;

        // Print the transformation
        printf ("Method #1: Applying transform\n");
        std::cout << rotation_translation_m << std::endl;
        std::cout << std::endl;
        std::cout << scale_m <<  std::endl;
        std::cout << std::endl;
        std::cout << final_m << std::endl;
        
        // You can either apply transform_1 or transform_2; they are the same
        pcl::PointCloud<pcl::PointXYZRGB> cloud; 
        pcl::fromPCLPointCloud2(mesh_binary_pcl.cloud, cloud); 
        pcl::transformPointCloud(cloud, cloud, final_m); 
        pcl::toPCLPointCloud2(cloud, mesh_binary_pcl.cloud);

        boost::filesystem::path p(inputfile);
        std::string writePath;
        if(DEBUG_ACTIVE){
            std::cout << "filename and extension : " << p.filename() << std::endl; // file.ext
            std::cout << "filename only          : " << p.stem() << std::endl;     // file
            std::cout << "parent directory       : " << p.parent_path() << std::endl;// directory less filename.ext
        }
        //writePath = "geo_OUTPUT.ply";
        writePath = p.parent_path().string()+boost::filesystem::path::preferred_separator+"geo"+p.filename().string();
        std::cout << "writing to:" << writePath << std::endl;
        pcl::io::savePLYFileBinary(writePath, mesh_binary_pcl);
        return 0;
    }
    
    if(pointcloud){
        // Load file | Works with PCD and PLY files
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    
        if (pcl::io::loadPLYFile ( inputfile, *source_cloud) < 0)  {
            std::cout << "Error loading point cloud " << inputfile << std::endl << std::endl;
            showHelp (argv[0]);
            return -1;
        }

        std::stringstream ss;
        std::ifstream file(jsonfile);
        if (file) {
            ss << file.rdbuf();
            file.close();
        } else {
            throw std::runtime_error("!! Unable to open json file");
        }
        
        Document document;
        document.Parse(ss.str().c_str());
        
        // Start parsing json string
        double scale = document["scale"].GetDouble();
        double translations[3];
        double rotations[3][3];
        
        Value& a = document["translation"];
        if(a.IsArray()){
            for (SizeType i = 0; i < a[0].Size(); i++){
                 double tmp = a[0][i].GetDouble();
                 //std::cout<<tmp<<", "<<std::endl;
                 translations[i] = tmp;
            }
        }
        
        Value& b = document["rotation"];
        if(b.IsArray()){
            for(int y=0;y<b.Size();y++){
                for(int i=0;i<b.Size();i++){
                    rotations[y][i] = b[y][i].GetDouble();
                }
            }
        }
        
        if(DEBUG_ACTIVE){
            std::cout << scale << std::endl;
            for(int x=0;x<3;x++){
                std::cout << translations[x] << " , ";
            }
            std::cout << std::endl;
            for(int x=0;x<3;x++){
                for(int y=0;y<3;y++){
                    std::cout << rotations[x][y] << " , ";
                }
                std::cout << std::endl;
            }
        }
        
        //exit(0);

        /* Reminder: how transformation matrices work :
        |-------> This column is the translation
        | 1 0 0 x |  \
        | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
        | 0 0 1 z |  /
        | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
        METHOD #1: Using a Matrix4f
        This is the "manual" method, perfect to understand but error prone !
        */
        Eigen::Matrix4f rotation_translation_m = Eigen::Matrix4f::Identity(),
            scale_m = Eigen::Matrix4f::Identity(),
            final_m = Eigen::Matrix4f::Identity();

        rotation_translation_m (0,0) = rotations[0][0];
        rotation_translation_m (0,1) = rotations[0][1];
        rotation_translation_m (0,2) = rotations[0][2];
        
        rotation_translation_m (1,0) = rotations[1][0];
        rotation_translation_m (1,1) = rotations[1][1];
        rotation_translation_m (1,2) = rotations[1][2];
        
        rotation_translation_m (2,0) = rotations[2][0];
        rotation_translation_m (2,1) = rotations[2][1];
        rotation_translation_m (2,2) = rotations[2][2];
        //    (row, column)

        // Define a translation of 2.5 meters on the x axis.
        rotation_translation_m (0,3) = translations[0];
        rotation_translation_m (1,3) = translations[1];
        rotation_translation_m (2,3) = translations[2];

        scale_m (0,0) = scale;
        scale_m (1,1) = scale;
        scale_m (2,2) = scale;

        // TransformedVector = TranslationMatrix * RotationMatrix * ScaleMatrix * Or0iginalVector;
        // scaling FIRST, and THEN the rotation, and THEN the translation
        final_m = rotation_translation_m * scale_m;

        // Print the transformation
        printf ("Method #1: Applying transform\n");
        std::cout << rotation_translation_m << std::endl;
        std::cout << std::endl;
        std::cout << scale_m <<  std::endl;
        std::cout << std::endl;
        std::cout << final_m << std::endl;

        // Executing the transformation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        // You can either apply transform_1 or transform_2; they are the same
        pcl::transformPointCloud (*source_cloud, *transformed_cloud, final_m);

        boost::filesystem::path p(inputfile);
        std::string writePath;
        if(DEBUG_ACTIVE){
            std::cout << "filename and extension : " << p.filename() << std::endl; // file.ext
            std::cout << "filename only          : " << p.stem() << std::endl;     // file
            std::cout << "parent directory       : " << p.parent_path() << std::endl;// directory less filename.ext
        }
        //writePath = "geo_OUTPUT.ply";
        writePath = p.parent_path().string()+boost::filesystem::path::preferred_separator+"geo"+p.filename().string();
        std::cout << "writing to:" << writePath << std::endl;
        pcl::io::savePLYFileBinary(writePath, *transformed_cloud);
        return 0;
    }
}
