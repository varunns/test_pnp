#include <iostream>

#include "EstimateControllerPose.h"

#include <source_location>
#include <cassert>


EstimateControllerPose* obj;

/*void test_input()
{
	std::cout<<"In test_input()"<<std::endl;
	if(obj->getImage().empty())
	{
		std::cout<<"The image is empty!!"<<std::endl;
	}
	else
	{
		std::cout<<"The image is good!!"<<std::endl;
	}
}

void test_contours()
{
	std::cout<<"In test_contours()"<<std::endl;
	obj->getFeatures();
	assert(obj->contours_.size() == 6);
	std::cout<<"Correct detection of contours"<<std::endl;
}

void test_rotatedAndSorted_contours()
{
	obj->rotateAndSortContours();
}*/

int main(int argc, char** argv)
{
	//cv::Mat image = cv::imread("../../data/latest.jpg");
	cv::Mat image = cv::imread("../../data/data_basic_test/pose1.jpg");
	obj = new EstimateControllerPose(image);
	obj->getPose();
/*	test_input();
	test_contours();
	test_rotatedAndSorted_contours();*/
	return 0;
}	