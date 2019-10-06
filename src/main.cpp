#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "read.h"

void GetQ(const Eigen::MatrixXd &calib_coods,
		  const Eigen::MatrixXd &p_w,
		  Eigen::MatrixXd & q){
	/*
	Use Calibrated Coods and P_W to construct Matrix Q.

	Arguments.
	Q           - The matrix that needs to be constructed
	calid_coods - contains the calibrated coods
	p_W         - contains the points in 3d coods.

	Qi =[X_w_i Y_w_i Z_w_i 1 0 0 0 0 -xi*X_w_i -xi*Y_w_i -xi*Z_w_i -xi
		 0 0 0 0 X_w_i Y_w_i Z_w_i 1 -yi*X_w_i -yi*Y_w_i -yi*Z_w_i -yi]

	*/
	
	// iterate over the 12 points
	for (int idx = 0 ; idx <12 ;idx++){

		// since once point formulates 2 points in Q

		// set first row according to the equation
		// Qi =[X_w_i Y_w_i Z_w_i 1 0 0 0 0 -xi*X_w_i -xi*Y_w_i -xi*Z_w_i -xi] if i is even
		q.row(2*idx)<<p_w.row(idx),1,
						0,0,0,0,
						-1*calib_coods(0,idx)*p_w.row(idx),-1*calib_coods(0,idx);
		
		// set the next row
		// Qi = [0 0 0 0 X_w_i Y_w_i Z_w_i 1 -yi*X_w_i -yi*Y_w_i -yi*Z_w_i -yi] if i is odd
		q.row(2*idx+1)<< 0, 0, 0, 0,
						 p_w.row(idx%12),1,
						 -1*calib_coods(1,idx)*p_w.row(idx),-1*calib_coods(1,idx);
		

	}


}


void GetCalibratedCoods(const Eigen::MatrixXd &k,
	                    const std::vector<double> &points,
	                    Eigen::MatrixXd &coods){
	/*
	This funciton changes coods to calibrated coordinates
	[x,y,1]^T = K^-1* [u,v,1]^T

	Arguments:
	k      - camera calibration matrix 
	points - u1, v1, u2, v2 ......
	coods  - Initally 0s (shape 3,12) need to convert into:
	         K^-1* [u,v,1]^T for all 12 points

	*/

	//initialize a 3x 12 matrix that stores the points in the image plane
	Eigen::MatrixXd image_coods(3,12);
	int cols = 0;

	for (int idx = 0; idx < points.size(); idx = idx+2){
		
		// fill image_coods up as [u,v,1]^T
		image_coods.col(cols++)<< points[idx],
							   points[idx+1],
							   1;	
		

	}

	// to convert into normalized coods multiply by K^-1
	coods = k.inverse()*image_coods;


}

Eigen::MatrixXd ComputeM(const Eigen::MatrixXd & Q){
	/*
	solving the over-determined system of equations
	Q.M = 0
	argmin ||Q.M|| subject to ||M|| = 1
	*/


	// compute  V matrix in svd 
	Eigen::JacobiSVD <Eigen::MatrixXd> svd (Q,Eigen::ComputeFullV);

	// get the last column that is M
	Eigen::MatrixXd M1 = svd.matrixV().col(svd.matrixV().cols()-1);

	// convert M to 4x3 matrix
	Eigen::Map<Eigen::MatrixXd> temp_M2 (M1.data(),4,3);

	// get transpose to change into 3xM
	Eigen::MatrixXd M2 = temp_M2.transpose();

	Eigen::MatrixXd M3;

	// check if det(R) is positive or negative
	if(M2.block(0,0,3,3).determinant()<0)
		// if negative multiply by -1
		M3 = -1*M2;
	else
		// if positive do nothing
		M3  = M2;

	// extract R 
	Eigen::MatrixXd R = M3.block(0,0,3,3);

	// find the closest orthogonal matrix to R
	Eigen::JacobiSVD <Eigen::MatrixXd> svd_r (R,Eigen:: ComputeFullU|Eigen::ComputeFullV);

	// compute r_tilde R̃ = U*V^T .
	Eigen::MatrixXd U = svd_r.matrixU();
	Eigen::MatrixXd V = svd_r.matrixV();
	Eigen::MatrixXd R_tilde = U*V.transpose();

	// calculate alpha
	double alpha = R_tilde.norm()/R.norm();



	// initalize M = [R_tilde , alpha*trans]
	Eigen::MatrixXd M4(3,4);
	M4.block(0,0,3,3)<<R_tilde;
	M4.col(3)<<alpha * M3.col(3);


	return M4;

}
std::vector<cv::Point> ReprojectPoints(const Eigen::MatrixXd &m,
									   const Eigen::MatrixXd &k,
									   const Eigen::MatrixXd &p){
	/*
	Convert points from 3d to image using the intrinsic and extrinsic parameters.

	Arguments:
	m - eigen matrix of shape 3*4 that contains that camera extrinsics
	k - eigen matrix of shape 3*3 that is the camera calib matrix
	p - eigen matrix that contains the 3d coods of the points in the world frame


	Returns:
	points - vector of cv::Point each point in has an .x and .y component  of p in the image coods
	*/

	// convert points from 3d to homogenous coods
	Eigen::MatrixXd ones = Eigen::MatrixXd::Constant(1,12,1.0);
	Eigen::MatrixXd points_homo(4,12);
	points_homo<<p.transpose(), ones;

	// in image coods
	Eigen::MatrixXd points_image = k*m*points_homo;


	std::vector<cv::Point> points;

	for (int c = 0;c<=points_image.cols(); c++){
		
		cv::Point temp;
		temp.x = points_image.coeff(0,c)/points_image.coeff(2,c);
		temp.y = points_image.coeff(1,c)/points_image.coeff(2,c);

		points.push_back(temp);
	
	}

	return points;
}





std::vector<cv::Point> GetCameraPoints(const std::vector<double> & points){

	/*
	Converts a vector of points to a vector cv::Points with Point.x and Point.y
	
	Arguments:
	points- vector of double of u1,v1, .........un, vn 

	Returns:
	p - vector of cv::Point each point in has an .x and .y component.

	*/

	std::vector<cv::Point> p;

	for (int idx = 0; idx <24; idx=idx+2){
		cv::Point temp;
		temp.x = points[idx];
		temp.y = points[idx+1];


		p.push_back(temp);
	}

	return p;
}


int main(){
	/* 
	k -  contains the camera calibration matrix

	p_w - contains the 12 positions in the wcs.

	detected_corners - contains m rows(m is the number of images). Each line i
	gives the 2d coordinate Pi = (Ui, Vi) of the projections of the 3d points
	in the undistorted image i given as a tuple (U1 , V1 , ..., Un , Vn ) ..
	*/

	//path to the files 
	std::string images_path = "/home/aun/Documents/visionalgorithms/exercise2/pnp/data/images_undistorted/";
	std::string image_type  = ".jpg"; 

	// get number of files 
	int number_images = GetNumberofImages(images_path, image_type);


	//read intrinsics of the camera
	std::string matrix_path = "/home/aun/Documents/visionalgorithms/exercise2/pnp/data/K.txt";
	Eigen::Matrix3d k;
	ReadCameraMatrix(matrix_path, k);


	//corners in the world cood system
	std::string corners_path = "/home/aun/Documents/visionalgorithms/exercise2/pnp/data/p_W_corners.txt";
	Eigen::MatrixXd p_w(12,3);
	ReadCornersinWCS(corners_path,p_w);

	//corners in the image
	std::string corner_path = "/home/aun/Documents/visionalgorithms/exercise2/pnp/data/detected_corners.txt";
	std::vector<std::vector<double>> detected_corners = GetCorners(corner_path);
	


	int fontFace = cv::FONT_HERSHEY_PLAIN;
  	double fontScale = 1;
	
	

	// read images 
	for (int image_idx = 1; image_idx <= number_images; image_idx++ ){

		// get path to the current image
		std::stringstream current_image_path;
		// append the necessary 0s and the image number.
		current_image_path<<images_path<<"img_"<<std::setfill('0')<<std::setw(4)<<image_idx<<image_type;


		// get calibrated/normalized coods
		Eigen::MatrixXd calibrated_coods(3,12);
		GetCalibratedCoods(k,detected_corners[image_idx-1], calibrated_coods);
		//get Q to solve Q.M = 0 
		Eigen::MatrixXd Q(24,12);
		GetQ(calibrated_coods,p_w,Q);
		
		//compute M
		Eigen::MatrixXd M = ComputeM(Q);



		//get reprojected points
		std::vector<cv::Point> points = ReprojectPoints(M,k,p_w);

		//get real points
		std::vector<cv::Point> orig_points = GetCameraPoints(detected_corners[image_idx-1]);


		//read image.
		cv::Mat image = cv::imread(current_image_path.str().c_str());
		cv::Mat orig  = image.clone();


		// draw shapes 

		// calculated
		for (auto x : points){
 			cv::putText(image,
 					   "X",
 					   x,
 					   fontFace,
 					   fontScale,
      				   cv::Scalar( 0, 0, 255),1);
 		}

 		// original
 		for (auto y : orig_points){
 			cv::putText(orig,
 					   "O",
 					   y,
 					   fontFace,
 					   fontScale,
      				   cv::Scalar( 255, 0, 0),1);
 		}


		//display image
		cv::imshow("Actual Points", orig);
		cv::imshow("Estimated Points", image);
		cv::waitKey(0);
	}






	return 0;


}