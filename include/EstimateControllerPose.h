#ifndef ESTIMATE_CONTROLLER_POSE_H_
#define ESTIMATE_CONTROLLER_POSE_H_

#include<iostream>
#include<vector>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "CameraModel.h"


class EstimateControllerPose
{
public:
	struct Pose
	{
		cv::Vec3f R;
		cv::Vec3f T;
	};

	struct ContourInfo
	{
		std::vector<cv::Point> contour;
		cv::Point2d center;
		cv::Point2d rotated_center;
		int id;
	};

	void getPose();
	EstimateControllerPose(const cv::Mat& image);


private:

	cv::Mat getImage()
	{
		return image_;
	}

	void getFeatures();

	std::vector<std::vector<cv::Point> > contours_;
	void rotateAndSortContours();

	std::vector<ContourInfo> contours_info_;
	std::vector<cv::Point2d> feature_points_;

	void getCandidatePointsAndPassToPnpVoting();
	

	std::vector<int> index_2d_;
	std::vector<int> index_3d_;

	std::vector<cv::Point3d> points_3d_solve_;
	std::vector<cv::Point2d> points_2d_solve_;

	std::vector<std::vector<int> > combinations_3d_;
	std::vector<cv::Point3d> led_points_3d_;

	bool static compareByX(const ContourInfo &a, const ContourInfo &b)
	{
    	return a.center.x < b.center.x;
	}
	
	bool debug_ = true;
	cv::Mat image_;
	
	void init();

	//solve p3p give 4 differnt solutions
	std::vector<cv::Mat> rotations_;
	std::vector<cv::Mat> translations_;
	void solvePnpKniep();
	void setup3dIndicesAnd2dIndicesPairs();


	//void getPoseForCombinationsofThree();
	//void checkPnPAndVote();

	CameraModel camera_model_obj_;

	void readCombinationsFromCSVFile(const std::string& str);

	void solvePnpAndVote();

	void test_inputs();

	void setVotes();

	Eigen::MatrixXd votes_;
};

#endif