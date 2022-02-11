/*-------------------------------------------------------------------*\

  NAME
    kitti2pcd


  DESCRIPTION
    Converts binary Point cloud files from the Kitti dataset 
    to a PCD ASCII format

    The KITTI dataset:
    http://www.cvlibs.net/datasets/kitti/

    PCD File format:
    http://pointclouds.org/documentation/tutorials/pcd_file_format.html

  AUTHOR
    Jari Honkanen

\*-------------------------------------------------------------------*/

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <algorithm>

namespace po = boost::program_options;
namespace boostfs = boost::filesystem;

typedef struct
{
    float x;
    float y;
    float z;
    float i;
} PointT;

enum OperModeE
{
    CONVERT_SINGLE_FILE = 0,
    CONVERT_DIRECTORY = 1
};

static OperModeE operMode = CONVERT_DIRECTORY;
static uint outputPrecision = 6;

/*---------------------------------------------------------------*\
    Read KITTI file
\*---------------------------------------------------------------*/
int readKittiFile(std::string inFileName, std::vector<PointT> &pointCloudVector, bool bDebugPrint)
{
    std::ifstream pcInFile(inFileName, std::ios::binary);
    if (!pcInFile)
    {
        std::cerr << "*** Error: readKittiFile(): Point Cloud file'" << inFileName << "' not found." << std::endl;
        return 0;
    }

    // get length of input file:
    pcInFile.seekg(0, pcInFile.end);
    int length = (int)pcInFile.tellg();
    pcInFile.seekg(0, pcInFile.beg);

    int numValues = length / sizeof(float);
    std::cout << "Number of values: " << numValues << std::endl;
    // Read data into buffer vector of floats
    std::vector<float> buffer(numValues);
    pcInFile.read(reinterpret_cast<char *>(&buffer[0]), numValues * sizeof(float));
    pcInFile.close();

    PointT point;
    std::vector<float> angles;
    for (uint32_t i = 0; i < buffer.size(); i += 5)
    {
        point.x = buffer[i];
        point.y = buffer[i + 1];
        point.z = buffer[i + 2];
        point.i = buffer[i + 3];

        float dist_xy = sqrt(point.x * point.x + point.y * point.y);
        float anglez_rad = atan2(point.z, dist_xy) * 180 / M_PI;
        if ((anglez_rad < -15.1) || (anglez_rad < -11.5 && anglez_rad > -12.5) || (anglez_rad < -9.5 && anglez_rad > -10.5) || (anglez_rad < -7.5 && anglez_rad > -8.5) || (anglez_rad < -5.2 && anglez_rad > -6.5) || (anglez_rad < -3.2 && anglez_rad > -4.8) || (anglez_rad < -1.2 && anglez_rad > -2.8) || (anglez_rad < -0.2 && anglez_rad > -0.8) || (anglez_rad > 0.1 && anglez_rad < 0.8) || (anglez_rad > 1.2 && anglez_rad < 1.8))
        {
            continue;
        }
        pointCloudVector.push_back(point);
        if (bDebugPrint)
        {
            angles.push_back(int(anglez_rad * 50) / 50.);
        }
        // std::cout << std::setprecision(outputPrecision) << point.x << " " << point.y << " " << point.z << " " << point.i << " " << dist_xy << " " << anglez_rad << std::endl;
    }
    if (bDebugPrint)
    {
        int c;
        std::sort(angles.begin(), angles.end()); //对于无序的数组需要先排序
        c = (std::unique(angles.begin(), angles.end()) - angles.begin());
        std::cout << "c = " << c << std::endl;
        //打印去重后的数组成员
        for (int i = 0; i < c; i++)
            std::cout << "a = [" << i << "] = " << angles[i] << std::endl;

        std::cout << "File '" << inFileName << "' contains " << length << " bytes and " << pointCloudVector.size() << " points" << std::endl;
    }

    return 1;
}

int WriteKittiFile(std::string outFileName, std::vector<PointT> &pointCloudVector, bool bDebugPrint)
{
    std::ofstream pcOutFile(outFileName, std::ios::binary | std::ios::out | std::ios::app);
    if (!pcOutFile)
    {
        std::cerr << "*** Error: openKittiFile():Couldn't open" << outFileName << "!" << std::endl;
        return 0;
    }

    // get length of input file:
    // pcInFile.seekg(0, pcInFile.end);
    // int length = (int)pcInFile.tellg();
    // pcInFile.seekg(0, pcInFile.beg);

    for (uint32_t i = 0; i < pointCloudVector.size(); i++)
    {
        pcOutFile.write((char *)&pointCloudVector[i].x, 3 * sizeof(float));
        pcOutFile.write((char *)&pointCloudVector[i].i, sizeof(float));

        // std::cout << std::setprecision(outputPrecision) << point.x << " " << point.y << " " << point.z << " " << point.i << " " << dist_xy << " " << anglez_rad << std::endl;
    }

    pcOutFile.close();

    return 1;
}

/*---------------------------------------------------------------*\
    Write PCD file
\*---------------------------------------------------------------*/
int writePcdFile(std::string outFileName, std::vector<PointT> &pointCloudVector, bool bDebugPrint)
{
    std::ofstream pcOutFile(outFileName);
    if (!pcOutFile)
    {
        std::cerr << "*** Error: writePcdFile(): Could not open output file '" << outFileName << "'" << std::endl;
        return 0;
    }

    // Write the PCD Header
    pcOutFile << "# .PCD v.7 - Point Cloud Data file format" << std::endl;
    pcOutFile << "VERSION .7" << std::endl;
    pcOutFile << "FIELDS x y z intensity" << std::endl;
    pcOutFile << "SIZE 4 4 4 4" << std::endl;
    pcOutFile << "TYPE F F F F" << std::endl;
    pcOutFile << "COUNT 1 1 1 1" << std::endl;
    pcOutFile << "WIDTH " << pointCloudVector.size() << std::endl;
    pcOutFile << "HEIGHT 1" << std::endl;
    pcOutFile << "POINTS " << pointCloudVector.size() << std::endl;
    pcOutFile << "DATA ASCII" << std::endl;
    pcOutFile << std::setprecision(outputPrecision);
    // Write points
    for (uint32_t i = 0; i < pointCloudVector.size(); i++)
    {
        pcOutFile << pointCloudVector[i].x << " " << pointCloudVector[i].y << " " << pointCloudVector[i].z << " " << pointCloudVector[i].i << std::endl;
    }

    if (bDebugPrint)
    {
        std::cout << "Wrote " << pointCloudVector.size() << " points to '" << outFileName << "'" << std::endl;
    }

    pcOutFile.close();

    return 1;
}

void resucurePath(boost::filesystem::path src_path, boost::filesystem::path dest_path)
{
  using namespace boost::filesystem;
  if (is_directory(src_path))
  {
    directory_iterator tmp_directory_end;
    directory_iterator tmp_dir_it(src_path);
 
    for (tmp_dir_it; tmp_dir_it != tmp_directory_end; tmp_dir_it++)
    {
      path child_dest_path = dest_path;
      if (is_directory(*tmp_dir_it))
      {
        std::string tmp_dir_name = (*tmp_dir_it).path().filename().string();
        child_dest_path.append(tmp_dir_name.begin(), tmp_dir_name.end());
 
        if (!exists(child_dest_path))
        {
          create_directory(child_dest_path);
        }
        std::cout << child_dest_path << std::endl;
      }
      else
      {
        std::string tmp_dir_name = (*tmp_dir_it).path().stem().string();
        child_dest_path.append(tmp_dir_name.begin(), tmp_dir_name.end());
      }
      resucurePath((*tmp_dir_it).path(), child_dest_path);
    }
  }
  else if (is_regular_file(src_path))
  {
    std::string src_path_string = src_path.string();
    std::string src_filename_string = src_path.filename().string();
    std::string src_parent_path_string = src_path.parent_path().string();
    std::string src_parent_dir_string = src_path.parent_path().filename().string();
    std::string dest_path_string = dest_path.parent_path().string()+"/"+src_filename_string;
    std::cout << "src_path_string: " << src_path_string << std::endl;
    std::cout << "src_filename_string: " << src_filename_string << std::endl;
    std::cout << "src_parent_path_string: " << src_parent_path_string << std::endl;
    std::cout << "src_parent_dir_string: " << src_parent_dir_string << std::endl;
    std::cout << "dest_path_string: " << dest_path_string << std::endl;
    if(src_parent_dir_string=="lidar_roof"){
        std::vector<PointT> pointCloudVector;
        readKittiFile(src_path_string, pointCloudVector, true);
        WriteKittiFile(dest_path_string, pointCloudVector, false);
    }
    // FILE *fp(NULL);
    // fp = fopen(dest_path.string().c_str(), "w+b");
    // fclose(fp);
    // fp = NULL;
    // std::cout <<src_path<<" "<< dest_path << std::endl;
  }
  
}

int main(int argc, char *argv[])
{

    /*------------------------------------------------------------------------*\
       Process Command Line Options
    \*------------------------------------------------------------------------*/

    // Optional options:
    po::options_description optsDesc("Optional Parameters");

    optsDesc.add_options()("precision,p", po::value<unsigned int>()->default_value(6), "Floating point precision for outputted values, default = 6")("help,h", "Print this help message");

    std::string srcPath;
    std::string destPath;

    // Required source directory or file
    po::options_description positional_params("Required Paramaters");
    positional_params.add_options()("src", po::value<std::string>(&srcPath)->required(), "Source Directory with KITTI bin files or a single KITTI bin file")("dest", po::value<std::string>(&destPath)->required(), "Destination Directory with PCD files or a single PCD file");

    optsDesc.add(positional_params);

    po::positional_options_description posOptsDesc;
    posOptsDesc.add("src", 1);  // Expect one argument
    posOptsDesc.add("dest", 1); // Expect one argument

    int unix_style = boost::program_options::command_line_style::unix_style |
                     boost::program_options::command_line_style::short_allow_next;

    po::variables_map vm;
    try
    {
        po::store(
            po::command_line_parser(argc, argv)
                .options(optsDesc)
                .positional(posOptsDesc)
                .style(unix_style)
                .run(),
            vm);

        po::notify(vm);

        if (argc < 3 || vm.count("help"))
        {
            std::cout << "USAGE: " << argv[0] << "\n"
                      << optsDesc << std::endl;
        }
    }
    catch (po::error &poe)
    {
        std::cerr << poe.what() << "\n"
                  << "USAGE: " << argv[0] << "\n"
                  << optsDesc << std::endl;

        return EXIT_FAILURE;
    }

    if (vm.count("precision"))
    {
        outputPrecision = vm["precision"].as<unsigned int>();
        std::cout << "Setting output precision to " << outputPrecision << std::endl;
    }

    // Check if 'srcPath' paramater is a file or a directory
    boostfs::path ps(srcPath);
    if (boostfs::exists(ps))
    {

        if (is_regular_file(ps))
        { // is source path just a single file?
            operMode = CONVERT_SINGLE_FILE;
        }
        else if (is_directory(ps))
        { // is source path a directory?
            operMode = CONVERT_DIRECTORY;
        }
        else
        {
            std::cerr << "*** ERROR: '" << ps << "' is not a regular file or directory!" << std::endl;
            std::cerr << "USAGE: " << argv[0] << "\n"
                      << optsDesc << std::endl;

            return EXIT_FAILURE;
        }
    }
    else
    {
        std::cerr << "*** ERROR: Source path '" << ps << "' does not exist!" << std::endl;
        std::cerr << "USAGE: " << argv[0] << "\n"
                  << optsDesc << std::endl;

        return EXIT_FAILURE;
    }

    // Check if destination directory exist
    if (operMode == CONVERT_DIRECTORY)
    {
        boostfs::path pd(destPath);
        if (is_regular_file(pd))
        { // Looking for a directory, put destPath is a file
            std::cerr << "*** ERROR: File '" << pd << "' is not a directory!" << std::endl;
            return EXIT_FAILURE;
        }
        if (!is_directory(pd))
        { // if desth path does not exist, create it
            boostfs::create_directories(pd);
        }
    }

    /*------------------------------------------------------------------------*\
       Read and Write Files
    \*------------------------------------------------------------------------*/
    std::vector<PointT> pointCloudVector;

    if (operMode == CONVERT_SINGLE_FILE)
    {
        readKittiFile(srcPath, pointCloudVector, true);
        WriteKittiFile(destPath, pointCloudVector, true);
        // writePcdFile(destPath, pointCloudVector, true);
    }
    else if (operMode == CONVERT_DIRECTORY)
    {

        resucurePath(boost::filesystem::path(srcPath), boost::filesystem::path(destPath));
       
    }

    return EXIT_SUCCESS;
}
