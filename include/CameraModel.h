#ifndef CAMERA_MODEL_H_
#define CAMERA_MODEL_H_

#include <opencv2/opencv.hpp>
#include <string>

class CameraModel
{
private:


	//void setCameraIntrinsicAndDistortionParameters();
	cv::Point2d getImagePoints(const cv::Point3d& pt);
	double r00, r01, r02,
	       r10, r11, r12,
	       r20, r21, r22;

	double t0, t1, t2;

	cv::Point3d rotatePoint(cv::Point3d pt);

	cv::Point3d translatePoint(cv::Point3d pt);

	void setTransformation(const cv::Mat& R, const cv::Mat& t);
	cv::Point2d pinholeProjection(const cv::Point3d& pt);
	cv::Point2d fisheyeDistortion(const cv::Point2d& pt);
	cv::Point2d getPixelCoordinates(const cv::Point2d& pt);

	double k1, k2, k3, k4, k5, k6; //distortion parameters

public:
		cv::Mat intrinsic_matrix = cv::Mat::zeros(3, 3, CV_64FC1);
	cv::Mat distCoeffs = cv::Mat(4,1,CV_64FC1);
	double fx, fy, cx, cy;   //intrinsics
	//CameraModel(const std::string& config_file);
	CameraModel();

	void undistortPoints(const std::vector<cv::Point2d>& distorted_points, 
						 std::vector<cv::Point2d>& undistort_points)
	{
		cv::fisheye::undistortPoints(distorted_points, 
									 undistort_points, 
									 intrinsic_matrix, 
									 distCoeffs, 
									 cv::noArray(), 
									 intrinsic_matrix);

		for(int i = 0; i < 3; i++)
		{
			std::cout<<undistort_points[i]<<std::endl; 
		}
	}

	void projectPoints(const std::vector<cv::Point3d>& points3, 
					   const cv::Mat& R, const cv::Mat& T, 
					   std::vector<cv::Point2d>& points2);


	cv::Point2d getImagePoint(const cv::Point3d& pt);

	double reprojectionError(std::vector<cv::Point2d>& pt1,
					  		 std::vector<cv::Point2d>& pt2);
};

#endif