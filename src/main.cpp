#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <point_types.h>

using namespace std;
using namespace pcl;

int main (int argc, char* argv[]) {
// Declare important variables
	string pcdFile;
	string csvFile;
	string timestamp;
	PointCloud<velodyne_pointcloud::PointXYZIR> cloud;
	ofstream outFile;

// Parse input
	if (argc==1) {
		cerr << "Too few input arguments!" << endl;
		cout << "Usage: pcd2csv  pcdInput.pcd" << endl;
		cout << "Exiting." << endl;
		return(-1);
	}
	else {
// Extract timestamp and store I/O filenames		
		pcdFile = argv[1];
		size_t type_idx = pcdFile.find_last_of(".");
		size_t dir_idx = pcdFile.find_last_of("/");
		string temp_str = pcdFile.substr(0, type_idx);
		timestamp = temp_str.substr(dir_idx+1, temp_str.length());
		csvFile = temp_str + ".csv";
	}

// Load in point cloud data
	cout << "Loading " << pcdFile << "..." << endl;
	int fileLoad = io::loadPCDFile<velodyne_pointcloud::PointXYZIR> (pcdFile, cloud);
	if (fileLoad == -1) {
		cerr << pcdFile << "not found. Exiting." << endl;
		return(-1);
	}
	else {
		cout << "File loaded. Converting..." << endl;
	}

// Store in file
	outFile.open(csvFile);
	outFile << "Points_m_XYZ:0," << "Points_m_XYZ:1," << "Points_m_XYZ:2,";
	outFile << "intensity," << "laser_id," << "azimuth," << "distance_m,";
	outFile << "adjustedtime," << "timestamp" << endl;
	for (size_t i = 0; i < cloud.width; ++i) {
		float x = cloud.points[i].x;
		float y = cloud.points[i].y;
		float z = cloud.points[i].z;
		float intensity = cloud.points[i].intensity;
		int ring = cloud.points[i].ring;
		float az = round(18000*(atan2(x,y)+M_PI)/M_PI);
		float dist = sqrt(x*x+y*y+z*z);
	    outFile << x << "," << y << "," << z << ",";
	    outFile << intensity << "," << ring << "," << az << "," << dist << ",";
	    outFile << timestamp << "," << timestamp << endl;
	}
	outFile.close();	
  cout << "Conversion done! Outputing: " << csvFile << endl;
  return (0);
}