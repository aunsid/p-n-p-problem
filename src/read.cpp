#include <glob.h>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <fstream>
#include <sstream>


std::vector<std::vector<double>> GetCorners(const std::string &file){

	std::ifstream infile(file);

	std::vector<std::vector<double>> corners;
	std::string line;
	double val;

	while(std::getline(infile, line)){
		std::stringstream streamed_line(line);
		std::vector<double> cor;
		while(streamed_line>>val){
			cor.push_back(val);
		}
		corners.push_back(cor);
	}


	return corners;

}

void ReadCornersinWCS(const std::string &file, Eigen::MatrixXd &points){
	std::ifstream infile(file);
	std::string line;
	int row_counter = 0;

	while(std::getline(infile, line)){
		std::stringstream streamed_line(line);
		char temp =' ';
		streamed_line>>points(row_counter,0)>>temp
					 >>points(row_counter,1)>>temp
					 >>points(row_counter,2);

		row_counter++;
	}
}


void ReadCameraMatrix(const std::string &file, Eigen::Matrix3d &k){

	std::ifstream infile(file);
	std::string line;
	int row_counter=0;

	while(std::getline(infile, line)){
		std::stringstream streamed_line(line);

		streamed_line>>k(row_counter,0)>>k(row_counter,1)>>k(row_counter,2);

		row_counter++;
	}



}



int GetNumberofImages(const std::string &path, const std::string  &type){
	/* 
	Get the path of the image and type of files to search

	Arguments:
	path - path in which to search for files
		   const string ref, as the file does not need to be changed.
	type -  extension of the file to look for  
		   const string  ref, does not need to be changed 


	Returns:
	number_images -  number of images found in the folder with "type" extension
			 int 
	*/

	glob_t gl;

 	size_t number_images = 0;
 	if(glob((path+"*"+type).c_str(), GLOB_NOSORT, NULL, &gl) == 0)
	  		number_images = gl.gl_pathc;


	// std::cout<<number_images<<std::endl;

	return (int)number_images;

}