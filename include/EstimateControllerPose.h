#ifndef ESTIMATE_CONTROLLER_POSE_H_
#define ESTIMATE_CONTROLLER_POSE_H_

#include<iostream>
#include<vector>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


class EstimateControllerPose
{
public:

	std::vector<std::vector<int> > point_3d_ids  {{1,2,3},
								    		 	 {2,3,4},
											     {3,4,5}};

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
	};

	bool static compareByX(const ContourInfo &a, const ContourInfo &b)
	{
    	return a.center.x < b.center.x;
	}
	
	EstimateControllerPose(){}
	Pose getPose(const cv::Mat& image);
	EstimateControllerPose(const cv::Mat& image);

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
	void solvePnPAndVote();

	std::vector<int> index_2d_;
	std::vector<int> index_3d_;

	void solvePnP

private:
	bool debug_ = true;
	cv::Mat image_;
	
	
	void init();
	//void getPoseForCombinationsofThree();
	//void checkPnPAndVote();

};

#endif