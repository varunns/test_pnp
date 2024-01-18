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
	std::cout<<"Number of 3d combinations: "<<combinations_3d_.size()<<std::endl;

}

/*void EstimateControllerPose::test_inputs()
{
	for(int i = 0; i < combinations_3d_.size(); i++)
	{
		for(int j = 0; j < 3; j++){
			std::cout<<combinations_3d_[i][j]<<"\t";
		}
		std::cout<<std::endl;
	}
	std::cout<<std::endl;

}*/


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
	cv::findContours( threshold_image, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE );

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
    	bounding_rect = cv::Rect( bounding_rect.x - 5, bounding_rect.y - 5, bounding_rect.width + 10, bounding_rect.height + 10 );
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

	std::cout<<"contours: "<<std::endl;
	if(debug_)
	{
		cv::Mat test_image = image_.clone();

		for(int i = 0; i < num_contours; i++)
		{
			std::cout<<contours_info_[i].center<<" "<<std::endl;
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


void EstimateControllerPose::getPose()
{
	getFeatures();
	rotateAndSortContours();
	setup3dIndicesAnd2dIndicesPairs();

	for(int i = 0; i < matching_stats.size(); i++)
	{
		for(int j = 0; j < matching_stats[i].matches.size(); j++)
		{
			std::cout<<" "<<j<<" "<<matching_stats[i].matches[j]<<std::endl;
		}
		std::cout<<"reproject_error: "<<matching_stats[i].reprojectionError<<std::endl;
		std::cout<<"votes: "<<matching_stats[i].votes<<std::endl;
		std::cout<<"R: "<<matching_stats[i].r<<std::endl;
		std::cout<<"T: "<<matching_stats[i].t<<std::endl;
	}

}

void EstimateControllerPose::setup3dIndicesAnd2dIndicesPairs()
{
	std::vector<int> indices_2d{0,1,2};
	std::vector<int> indices_3d{0,1,2};

	solvePnpKniep(indices_2d, indices_3d);
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


void EstimateControllerPose::doVotingForIncreasing(cv::Mat r, cv::Mat T, 
						   						   std::vector<int> index3, 
						   						   std::vector<int> index2, 
						   						   double reproject_error)
{ 
	std::cout<<"Main indicess: "<<std::endl;

	std::vector<int> matches(contours_.size(), -1);

	for(int i = 0; i < 3; i++)
	{
		matches[index2[i]] = index3[i];
	}

	std::cout<<"Matches before: "<<std::endl;
	for(int i = 0; i < matches.size(); i++)
	{	
		std::cout<<i<<" "<<matches[i]<<std::endl;
	}

	int count_less = 0;
	int start_index_2d = index2[0] - 1;
	int start_index_3d = index3[0] - 1;
	//int start_index_3d = min_index - 1;
	//int start_index_3d = min_index - 1;
	//int count = 0;
	cv::Mat local_image_ = image_.clone();
	double distance_less = 0;
	while(start_index_2d > -1 && start_index_3d > -1)
	{
		std::cout<<"test index: "<<start_index_2d<<" "<<start_index_3d<<std::endl;
		cv::Point2d pt2 = contours_info_[start_index_2d].center;
		cv::Point2d pt2_projected = camera_model_obj_.getImagePoint(led_points_3d_[start_index_3d], r, T);

		std::cout<<"proj: "<<pt2<<" "<<pt2_projected<<std::endl;

		cv::circle(local_image_, pt2, 5, cv::Scalar(0,0,255), -1);
		cv::circle(local_image_,  pt2_projected, 1, cv::Scalar(255, 0, 0), -1);


		cv::Rect bound_rect = cv::boundingRect(contours_info_[start_index_2d].contour);
		bound_rect = cv::Rect(bound_rect.x - 5, bound_rect.y - 5, bound_rect.width + 10, bound_rect.height + 10);


	///	double score = cv::pointPolygonTest(contours_info_[start_index_2d].contour, pt2_projected, false);

		if(rectContains(bound_rect, pt2_projected) && matches[start_index_2d] != -1);
		{
			count_less++;
			distance_less += cv::norm(pt2 - pt2_projected);
			matches[start_index_2d] = start_index_3d;
			votes_(start_index_2d, start_index_3d)++;
		}

		start_index_2d--;
		start_index_3d--;
	}
	std::cout<<"count_less: "<<count_less<<std::endl;
	
	int count_more = 0;

	//int max_index = *std::max_element(combinations_3d[index3].begin(), combinations_3d[index3].end());
	start_index_2d = index2.back() + 1;
	start_index_3d = index3.back() + 1;
	//start_index_3d = max_index + 1;

	double distance_more = 0;
	while(start_index_2d < contours_info_.size() && start_index_3d < led_points_3d_.size())
	{
		std::cout<<"test index: "<<start_index_2d<<" "<<start_index_3d<<std::endl;
		cv::Point2d pt2 = contours_info_[start_index_2d].center;
		cv::Point2d pt2_projected = camera_model_obj_.getImagePoint(led_points_3d_[start_index_3d], r, T);

		std::cout<<"proj: "<<pt2<<std::endl;

		cv::circle(local_image_, pt2, 5, cv::Scalar(0,0,255), 1);
		cv::circle(local_image_,  pt2_projected, 1, cv::Scalar(255, 0, 0), -1);


		cv::Rect bound_rect = cv::boundingRect(contours_info_[start_index_2d].contour);
		bound_rect = cv::Rect(bound_rect.x - 5, bound_rect.y - 5, bound_rect.width + 10, bound_rect.height + 10);

		
		//double score = cv::pointPolygonTest(contours_info_[start_index_2d].contour, pt2_projected, false);

		if(rectContains(bound_rect, pt2_projected) && matches[start_index_2d] != -1);
		{
			count_more++;
			distance_more += cv::norm(pt2 - pt2_projected);
			matches[start_index_2d] = start_index_3d;
			votes_(start_index_2d, start_index_3d)++;
		}


		start_index_2d++;
		start_index_3d++;
	}



	std::cout<<"count_more: "<<count_more<<std::endl;

	cv::namedWindow("local", cv::WINDOW_NORMAL);
	cv::imshow("local", local_image_);
	cv::waitKey(0);
	std::cout<<"before finished voting"<<std::endl;
	local_image_.release();

/*	std::cout<<"count: "<<count<<std::endl;

	if(count > 0)
	{
		votes(combinations_2d[index2][0], combinations_3d[index3][0])++;
		votes(combinations_2d[index2][1], combinations_3d[index3][1])++;
		votes(combinations_2d[index2][2], combinations_3d[index3][2])++;
	}*/

	std::cout<<"Matches: "<<std::endl;
	for(int i = 0; i < matches.size(); i++)
	{	
		std::cout<<i<<" "<<matches[i]<<std::endl;
	}

	MatchingStat temp;
	temp.index3d = index3;
	temp.index2d = index2;
	temp.r = r;
	temp.t = T;
	temp.reprojectionError = (reproject_error*3 +distance_less + distance_more)/(3+count_less+count_more);
	temp.votes = 3 + count_more + count_less;
	temp.matches = matches;

	matching_stats.push_back(temp);

}

/*
void doVotingForDecreasing(cv::Mat r, cv::Mat T, int index3, int index2, std::vector<cv::Point2d>& projectedPoints, int& count)
{
	//int max_index = *std::max_element(combinations_3d[index3].begin(), combinations_3d[index3].end());
	int start_index_2d = combinations_2d[index2][0] - 1;
	int start_index_3d = combinations_3d[index3].back() + 1;
	//int start_index_3d = max_index + 1;
	//int count = 0;
	while(start_index_2d > -1 && start_index_3d < led_points_3d.size())
	{
		std::cout<<"test index: "<<start_index_2d<<" "<<start_index_3d<<std::endl;
		cv::Point2d pt2 = all_detected_leds[start_index_2d];
		cv::Point2d pt2_projected = getImagePoint(led_points_3d[start_index_3d], r, T);

		double score = cv::pointPolygonTest(contours_stats[start_index_2d].contour, pt2_projected, false);
		projectedPoints.push_back(pt2_projected);

		if(score != -1)
		{
			count++;
			votes(start_index_2d, start_index_3d)++;
		}

		std::cout<<"Score:"<<score<<std::endl;

		start_index_2d--;
		start_index_3d++;
	}

	//int min_index = *std::min_element(combinations_3d[index3].begin(), combinations_3d[index3].end());
	start_index_2d = combinations_2d[index2].back() + 1;
	start_index_3d = combinations_3d[index3][0] - 1;
	//start_index_3d = min_index - 1;
	while(start_index_2d < all_detected_leds.size() && start_index_3d > -1)
	{
		std::cout<<"test index: "<<start_index_2d<<" "<<start_index_3d<<std::endl;
		cv::Point2d pt2 = all_detected_leds[start_index_2d];
		cv::Point2d pt2_projected = getImagePoint(led_points_3d[start_index_3d], r, T);
		projectedPoints.push_back(pt2_projected);

		double score = cv::pointPolygonTest(contours_stats[start_index_2d].contour, pt2_projected, false);

		if(score != -1)
		{
			count++;
			votes(start_index_2d, start_index_3d)++;
		}

		std::cout<<"Score:"<<score<<std::endl;

		start_index_2d++;
		start_index_3d--;
	}

	std::cout<<"before finished voting"<<std::endl;

}
*/

void EstimateControllerPose::solvePnpKniep(std::vector<int> indices_2d,
										   std::vector<int> indices_3d)
{
	TooN::Matrix<3,3> image_vectors;
	TooN::Matrix<3,3> world_points;
	TooN::Matrix<3,16> solutions;

	std::vector<cv::Point2d> points_2d;
	std::vector<cv::Point3d> points_3d;

	for(int i = 0; i < 3; i++)
	{
		points_2d.push_back(contours_info_[indices_2d[i]].center);
		points_3d.push_back(led_points_3d_[indices_3d[i]]);
	}

	std::vector<cv::Point2d> undistort_image_points;
	camera_model_obj_.undistortPoints(points_2d, undistort_image_points);

	for(int i = 0; i < 3; i++)
	{
		std::cout<<points_3d[i]<<" "<<points_2d[i]<<std::endl;
	}

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

		/*std::cout<<R<<std::endl;
		std::cout<<T<<std::endl;*/
		
		//check isnan
	/*	if(checkIsNaN(R, T))
		{
			continue;
		}*/

		//invert to get leds ot get their pose
		cv::Mat r_led = R.inv();
		cv::Mat t_led = -R.inv()*T;

		std::cout<<r_led<<std::endl;
		std::cout<<t_led<<std::endl;

		//check negative z
		if(t_led.at<double>(2,0) < 0)
		{
			continue;
		}

		//check minimum limits for distance
		cv::Mat sq = t_led.t()*t_led;
		double distance_led = std::sqrt(sq.at<double>(0,0));
		std::cout<<"distance_led: "<<distance_led<<std::endl;
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

		std::cout<<"projected_points: "<<projected_points.size()<<std::endl;
		std::cout<<"reproject_error: "<<reproject_error<<std::endl;
		cv::Mat test_image = image_.clone();
		for(int j = 0; j < projected_points.size(); j++)
		{
			cv::circle(test_image, projected_points[i], 1, cv::Scalar(0,0,255), -1);
		}



		doVotingForIncreasing(r_led, t_led, 
							  indices_3d, indices_2d, reproject_error);

		cv::namedWindow("test11", cv::WINDOW_NORMAL);
		cv::imshow("test11", test_image);

		cv::waitKey(0);

		R.release();
		T.release();
		r_led.release();
		t_led.release();
	}

}

