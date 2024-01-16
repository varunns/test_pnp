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
		//cv::waitKey(0);
	}
	readCombinationsFromCSVFile("/home/varun/dev/test_pnp/data/combination.csv");
	Constants const_obj;
	led_points_3d_ = const_obj.getPoints3d();
	std::cout<<combinations_3d_.size()<<std::endl;

}

void EstimateControllerPose::test_inputs()
{
	for(int i = 0; i < combinations_3d_.size(); i++)
	{
		for(int j = 0; j < 3; j++){
			std::cout<<combinations_3d_[i][j]<<"\t";
		}
		std::cout<<std::endl;
	}
	std::cout<<std::endl;

}


void EstimateControllerPose::readCombinationsFromCSVFile(const std::string& str)
{
    std::ifstream csvFile;
    csvFile.open(str.c_str());

	std::string line;
	std::vector<std::string> vec;
	
	while(getline(csvFile, line))
	{
		if(line.empty())
		{
			continue;
		}
		std::istringstream iss(line);
		std::string lineStream;
		std::vector<int> row;
		while(getline(iss, lineStream, ','))
		{
			row.push_back(stoi(lineStream));
		}
		combinations_3d_.push_back(row);
	}
}

void EstimateControllerPose::getFeatures()
{
	cv::Mat test_image = image_.clone();
	std::vector<cv::Vec4i> hierarchy;
	cv::Mat gray_image, threshold_image;
	cv::cvtColor(test_image, gray_image, cv::COLOR_BGR2GRAY);
	cv::threshold(gray_image, threshold_image, 75, 255, 0);
	cv::findContours( threshold_image, contours_, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

	for(int i = 0; i < contours_.size(); i++)
	{
		cv::Moments mu;
    	mu = cv::moments(contours_[i], false);
    	ContourInfo contour_info_temp;
    	contour_info_temp.contour = contours_[i];
    	contour_info_temp.center = cv::Point2d(mu.m10 / mu.m00, mu.m01 / mu.m00);
    	if(std::isnan(contour_info_temp.center.x) || 
    	   std::isnan(contour_info_temp.center.y))
    	{
    		continue;
    	}
    	cv::Rect bounding_rect = cv::boundingRect(contours_[i]);
    	bounding_rect = cv::Rect( bounding_rect.x - 3, bounding_rect.y - 3, bounding_rect.width + 6, bounding_rect.height + 6 );
    	contour_info_temp.bounding_rect = bounding_rect;

    	contours_info_.push_back(contour_info_temp);

    	//std::cout<<contour_info_temp.center<<std::endl;
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
	}

	//initialize vote matrix
	votes_ = Eigen::MatrixXd::Zero(contours_info_.size(), led_points_3d_.size());
}

void EstimateControllerPose::getPose()
{
	getFeatures();
	rotateAndSortContours();
	setup3dIndicesAnd2dIndicesPairs();

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

bool EstimateControllerPose::rectContains(cv::Rect rect, cv::Point2d pt)
{
	double x1 = rect.x; double y1 = rect.y; double x2 = x1+rect.width; double y2 = y1+rect.height;

	if(pt.x < x1 || pt.y < y1 || pt.x > x2 || pt.y > y2)
	{
		return false;
	}

	return true;
}

void EstimateControllerPose::performNeighborLedMatching(MatchingStat& matching_stat)
{
	int min_index = std::max(*std::min(matching_stat.index3d.begin(), matching_stat.index3d.end()) - 3, 0);
	int max_index = std::min(*std::max(matching_stat.index3d.begin(), matching_stat.index3d.end()) + 3, 14);

	std::cout<<min_index<<" "<<max_index<<std::endl;

	cv::Mat r = matching_stat.r;
	cv::Mat t = matching_stat.t;

	cv::Mat test_image = image_.clone();

	for(int i = min_index; i < max_index; i++)
	{
		//check index is not in 3d indices
		if(std::find(matching_stat.index3d.begin(), matching_stat.index3d.end(), i) != matching_stat.index3d.end())
		{
			continue;
		}
		//for each point get image point
		cv::Point2d pt = camera_model_obj_.getImagePoint(led_points_3d_[i], r, t);
		cv::circle(test_image, pt, 1, cv::Scalar(0,0,255), -1);

		//loop in all 2d indices
		for(int j = 0; j < contours_info_.size(); j++)
		{
			//check if the index is not in existing indices
			if(std::find(matching_stat.index2d.begin(), matching_stat.index2d.end(), j) != matching_stat.index2d.end())
			{
				continue;
			}
			//check with rect
			bool is_point_near_contour = rectContains(contours_info_[i].bounding_rect, pt);
			if(is_point_near_contour && matching_stat.full_matches[j] != -1)
			{
				matching_stat.full_matches[j] = i;
				matching_stat.votes++;
				continue;
			}
			//if the rect already has point continue
		}
		//update full_match array
		//add reprojection error
		//loop
	}

	cv::namedWindow("project", cv::WINDOW_NORMAL);
	cv::imshow("porject", test_image);
	cv::waitKey(0);

	for(int i = 0; i < matching_stat.full_matches.size(); i++)
	{
		std::cout<<matching_stat.full_matches[i]<<"\t";
	}
	std::cout<<std::endl;
}

void EstimateControllerPose::setup3dIndicesAnd2dIndicesPairs()
{
	std::vector<cv::Point2d> points_2d;

	int mid_point_index = contours_info_.size()/2;
	points_2d.push_back(contours_info_[mid_point_index-1].center);
	points_2d.push_back(contours_info_[mid_point_index].center);
	points_2d.push_back(contours_info_[mid_point_index+1].center);

	std::vector<int> indices_2d;
	indices_2d.push_back(mid_point_index-1);
	indices_2d.push_back(mid_point_index);
	indices_2d.push_back(mid_point_index+1);

	for(int i = 0; i < combinations_3d_.size(); i++)
	{
		solvePnpKniep(combinations_3d_[i], indices_2d, points_2d);
	}

	int matching_stats_size = matching_stats.size();
	for(int i = 0; i < matching_stats_size; i++)
	{
		performNeighborLedMatching(matching_stats[i]);
		for(int j = 0; j < matching_stats[i].full_matches.size(); j++)
		{
			std::cout<<j<<" "<<matching_stats[i].full_matches[j]<<std::endl;
		}
		std::cout<<"-----------------------------------------------------------------------------------"<<std::endl;
	}
}

/*void EstimateControllerPose::solvePnpAndVote()
{
	solvePnpKniep();
	getIdsFromVotes();
}*/

void EstimateControllerPose::getIdsFromVotes()
{

}

void EstimateControllerPose::drawPoints(const std::vector<cv::Point2d>& points2)
{
	cv::Mat test_image = image_.clone();
	for(int i = 0; i < points2.size(); i++)
	{
		cv::circle(test_image, points2[i], 1, cv::Scalar(0,0,255), -1);
	}
	cv::namedWindow("porject", cv::WINDOW_NORMAL);
	cv::imshow("porject", test_image);
	cv::waitKey(0);
}


void EstimateControllerPose::setVotes(const cv::Mat& rvec,
									  const cv::Mat& tvec)
{
	cv::Mat r = rvec.inv();
	cv::Mat t = -r*tvec;
	std::vector<cv::Point2d> projected_points;	

	camera_model_obj_.projectPoints(points_3d_solve_,
									r, t,
									projected_points);

/*	for(int i = 0; i < 3; i++)
	{
		std::cout<<points_3d_solve_[i]<<" "<<projected_points[i]<<std::endl;
	}*/

	drawPoints(projected_points);

	if(camera_model_obj_.reprojectionError(points_2d_solve_, projected_points) > 1)
	{
		r.release();
		t.release();
		return;
	}

	//std::cout<<"I am past reprojection"<<std::endl;
	//getVotesForTestFeatures(r, t);
	int diff = index_3d_[2] - index_3d_[0];
	int increasing = 1;

	int count = 0;

	int max_index = *std::max(index_3d_.begin(), index_3d_.end());
	int min_index = *std::min(index_3d_.begin(), index_3d_.end());

 
	if(diff > 0)
	{
		int low_side_3d  = index_3d_[0] - 1;
		int high_side_3d = index_3d_.back() + 1;
	/*	int low_side_3d  = min_index - 1;
		int high_side_3d = max_index + 1;*/
		getVotes(rvec, tvec, low_side_3d,
				 high_side_3d, -1,
				 led_points_3d_.size(),
				 increasing,
				 count);
	}
	else
	{
		int low_side_3d = index_3d_.back() + 1;
		int high_side_3d = index_3d_[0] - 1;
	/*	int low_side_3d = max_index + 1;
		int high_side_3d = min_index - 1;*/
		increasing = -1;
		getVotes(rvec, tvec, low_side_3d,
				 high_side_3d, led_points_3d_.size(),
				 -1, increasing,
				 count);
	}

}




void EstimateControllerPose::getVotes(const cv::Mat& r,
								      const cv::Mat& t,
								      const int low_side,
								      const int high_side,
								      const int low_limit,
								      const int high_limit,
								      const int sign,
								      int& count
								      )
{
	int start_index_2d = index_2d_[0] - 1;
	int start_index_3d = low_side;
	while(start_index_2d > -1 && sign*start_index_3d > sign*low_limit)
	{
		//std::cout<<start_index_3d<<" "<<start_index_2d<<std::endl;

		cv::Point2d pt2 = contours_info_[start_index_2d].center;
		cv::Point2d pt2_projected = camera_model_obj_.getImagePoint(led_points_3d_[start_index_3d]);

		double score = cv::pointPolygonTest(contours_info_[start_index_2d].contour, pt2_projected, false);
		//projectedPoints.push_back(pt2_projected);

		if(score != -1)
		{
			count++;
			votes_(start_index_2d, start_index_3d)++;
		}

		//std::cout<<"Score:"<<score<<std::endl;

		start_index_2d--;
		start_index_3d = start_index_3d - sign*1;
	}

	start_index_2d = index_2d_.back() + 1;
	start_index_3d = high_side;
	while(start_index_2d < contours_info_.size() && sign*start_index_3d < sign*high_limit)
	{
		//std::cout<<start_index_3d<<" "<<start_index_2d<<std::endl;

		cv::Point2d pt2 = contours_info_[start_index_2d].center;
		cv::Point2d pt2_projected = camera_model_obj_.getImagePoint(led_points_3d_[start_index_3d]);

		double score = cv::pointPolygonTest(contours_info_[start_index_2d].contour, pt2_projected, false);

		if(score != -1)
		{
			count++;
			votes_(start_index_2d, start_index_3d)++;
		}

		//std::cout<<"Score:"<<score<<std::endl;


		start_index_2d++;
		start_index_3d = start_index_3d + sign*1;
	}

	if(count > 0)
	{
		votes_(index_2d_[0], index_3d_[0])++;
		votes_(index_2d_[1], index_3d_[1])++;
		votes_(index_2d_[2], index_3d_[2])++;
	}

}

bool EstimateControllerPose::checkIsNaN(cv::Mat r, cv::Mat t)
{
	if(std::isnan(t.at<double>(0,0)) || std::isnan(t.at<double>(1,0)) ||std::isnan(t.at<double>(2,0)) )
	{
		return true;
	}

	for(int j = 0; j < 3; j++)
	{
		for(int k = 0; k < 3; k++)
		{
			if( std::isnan(r.at<double>(j,k)))
			{
				return true;
			}
		}
	}

	return false;

}


void EstimateControllerPose::solvePnpKniep(std::vector<int> indices_3d,
										   std::vector<int> indices_2d,
										   std::vector<cv::Point2d> points_2d)
{

	std::vector<cv::Point3d> points_3d;

	for(int i = 0; i < 3; i++)
	{
		points_3d.push_back(led_points_3d_[indices_3d[i]]);
		//cv::circle(test_image1, points_2d[i], 1, cv::Scalar(0,255,0), -1);
	}

	TooN::Matrix<3,3> image_vectors;
	TooN::Matrix<3,3> world_points;
	TooN::Matrix<3,16> solutions;

	std::vector<cv::Point2d> undistort_image_points;
	camera_model_obj_.undistortPoints(points_2d, undistort_image_points);

	Eigen::Vector3d single_vector;
	//setup vector for solvepnp
	for(int i = 0; i < 3; i++)
	{
		single_vector(0) = (undistort_image_points[i].x - camera_model_obj_.intrinsic_matrix.at<double>(0, 2)) / camera_model_obj_.intrinsic_matrix.at<double>(0, 0);
		single_vector(1) = (undistort_image_points[i].y - camera_model_obj_.intrinsic_matrix.at<double>(1, 2) ) / camera_model_obj_.intrinsic_matrix.at<double>(1, 1);
		single_vector(2) = 1;
		Eigen::Vector3d temp = single_vector/single_vector.norm();

		image_vectors(0,i) = temp[0];
		image_vectors(1,i) = temp[1];
		image_vectors(2,i) = temp[2];

		world_points(0,i) = points_3d[i].x;
		world_points(1,i) = points_3d[i].y;
		world_points(2,i) = points_3d[i].z;	
	}

	P3p obj;
	int message = obj.computePoses(image_vectors, world_points, solutions);

	for(int i = 0; i < 16; i = i+4)
	{	

		TooN::Matrix<3,1> Ttoon = solutions.slice(0, i, 3, 1);
		TooN::Matrix<3,3> Rtoon = solutions.slice(0, i+1, 3, 3);

		cv::Mat T = cv::Mat::zeros(3, 1, CV_64FC1);

		for(int j = 0; j < 3; j++)
		{
			T.at<double>(j,0) = Ttoon[j][0];
		}

		cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);

		for(int j = 0; j < 3; j++)
		{
			for(int k = 0; k < 3; k++)
			{
				R.at<double>(j,k) = Rtoon[j][k];
			}
		}

	/*	std::cout<<R<<std::endl;
		std::cout<<T<<std::endl;*/
		
		//check isnan
		if(checkIsNaN(R, T))
		{
			continue;
		}

		//invert to get leds ot get their pose
		cv::Mat r_led = R.inv();
		cv::Mat t_led = -R.inv()*T;

		//check negative z
		if(t_led.at<double>(2,0) < 0)
		{
			continue;
		}

		//check minimum limits for distance
		cv::Mat sq = t_led.t()*t_led;
		double distance_led = std::sqrt(sq.at<double>(0,0));
		if(distance_led < MIN_DISTANCE_LED || distance_led > MAX_DISTANCE_LED)
		{
			continue;
		}

		//check reprojection error
		std::vector<cv::Point2d> projected_points;
		camera_model_obj_.projectPoints(points_3d, r_led, t_led, projected_points);
		double reproject_error = camera_model_obj_.reprojectionError(points_2d, projected_points);
		if(reproject_error > 1.0)
		{
			continue;
		}
		

		//start voting and fill the struct
		MatchingStat temp;
		temp.index2d = indices_2d;
		temp.index3d = indices_3d;
		temp.r = r_led;
		temp.t = t_led;
		std::vector<int> temp_match(contours_info_.size(), -1);
		temp.reprojectionError = reproject_error;
		temp.votes = 3;
		temp.full_matches = temp_match;
		for(int i = 0; i < 3; i++)
		{
			temp.full_matches[indices_2d[i]] = indices_3d[i];
		}
		matching_stats.push_back(temp);

		cv::Mat test_image1 = image_.clone();
		for(int j = 0; j < 3; j++)
		{
			cv::circle(test_image1, projected_points[j], 1, cv::Scalar(0,0,255), -1);
		}
		cv::namedWindow("porject1", cv::WINDOW_NORMAL);
		cv::imshow("porject1", test_image1);
		//cv::waitKey(0);
		test_image1.release();

		//count votes

		R.release();
		T.release();
	}

}

