#include "EstimateControllerPose.h"
#include "Constants.h"
#include "P3p.h"
#include <TooN/TooN.h>

EstimateControllerPose::EstimateControllerPose(const cv::Mat& image)
{
	image_ = image;
	if(debug_)
	{
		cv::namedWindow("input", cv::WINDOW_NORMAL);
		cv::imshow("input", image_);
		cv::waitKey(0);
	}
	init();
}

void EstimateControllerPose::init()
{
	
}

void EstimateControllerPose::getFeatures()
{
	cv::Mat test_image = image_.clone();
	std::vector<cv::Vec4i> hierarchy;
	cv::Mat gray_image, threshold_image;
	cv::cvtColor(test_image, gray_image, cv::COLOR_BGR2GRAY);
	cv::threshold(gray_image, threshold_image, 100, 255, 0);
	cv::findContours( threshold_image, contours_, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

	for(int i = 0; i < contours_.size(); i++)
	{
		cv::Moments mu;
    	mu = cv::moments(contours_[i], false);
    	ContourInfo contour_info_temp;
    	contour_info_temp.contour = contours_[i];
    	contour_info_temp.center = cv::Point2d(mu.m10 / mu.m00, mu.m01 / mu.m00);
    	contours_info_.push_back(contour_info_temp);
	}

	if(debug_)
	{
		cv::Mat drawing = cv::Mat::zeros( test_image.size(), CV_8UC3 );
	    for( size_t i = 0; i< contours_.size(); i++ )
	    {
	    	cv::Scalar color(0,0,255);
	        cv::drawContours( drawing, contours_, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
	    }
	    cv::namedWindow("Contours", cv::WINDOW_NORMAL);
    	cv::imshow( "Contours", drawing );
    	cv::waitKey(0);
	}
}

void EstimateControllerPose::rotateAndSortContours()
{
	double MIN_X = std::numeric_limits<double>::max();
	double MAX_X = std::numeric_limits<double>::min();
	double MIN_Y = std::numeric_limits<double>::max();
	double MAX_Y = std::numeric_limits<double>::min();

	size_t num_contours = contours_.size();
	for(size_t i = 0; i < num_contours; i++)
	{
		MIN_X = std::min(MIN_X, contours_info_[i].center.x);
		MAX_X = std::max(MAX_X, contours_info_[i].center.x);
		MIN_Y = std::min(MIN_X, contours_info_[i].center.y);
		MAX_Y = std::max(MAX_X, contours_info_[i].center.y);
	}

	float theta;
	if(MIN_X == MAX_X)
	{
		theta = CV_PI/2;
	}
	else
	{
		theta = std::atan((MAX_Y - MIN_Y)/(MAX_X - MIN_X));
	}

	for(size_t i = 0; i < num_contours; i++)
	{
		double x = contours_info_[i].center.x;
		double y = contours_info_[i].center.y;
		contours_info_[i].rotated_center.x = x*std::cos(theta) - y*std::sin(theta);
		contours_info_[i].rotated_center.y = x*std::cos(theta) + y*std::cos(theta);
	}

	std::sort(contours_info_.begin(), contours_info_.end(), compareByX);

	for(int i = 0; i < contours_info_.size(); i++)
	{
		contours_info_[i].id = i;
	}

	if(debug_)
	{
		cv::Mat test_image = image_.clone();

		for(int i = 0; i < num_contours; i++)
		{
			cv::circle(test_image, contours_info_[i].center, 3, cv::Scalar(0,0,255), -1);
			cv::putText(test_image, std::to_string(i), 
						contours_info_[i].center,  
						cv::FONT_HERSHEY_DUPLEX,
						1.0,
						CV_RGB(118, 185, 0), //font color
            			1);
		}
		cv::namedWindow("rotated Contours", cv::WINDOW_NORMAL);
    	cv::imshow( "rotated Contours", test_image );
    	cv::waitKey(0);
	}
}

void EstimateControllerPose::getCandidatePointsAndPassToPnpVoting()
{
	for(int i = 0; i < point_3d_ids.size(); i++)
	{
		index_3d_ = point_3d_ids[i];
		for(int j = 0; j < contours_.size()-3; j++)
		{
			index_2d_.push_back(j);
			index_2d_.push_back(j+1);
			index_2d_.push_back(j+2);
			//solvePnPAndVote();
		}
		index_2d_.clear();
		index_3d_.clear();
	}
}

void EstimateControllerPose::solvePnpKniep(const std::vector<cv::Point3d>& points3d,
										   const std::vector<cv::Point2d>& points2d,
										   Pose& pose_estimate)
{

}