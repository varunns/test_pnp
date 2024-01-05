#include "CameraModel.h"

CameraModel::CameraModel(const std::string& config_path)
{
  	intrinsic_matrix.at<double>(0, 0) = 235.483; //319.59208146;
  	intrinsic_matrix.at<double>(1, 1) = 235.361; //237.53868415;
  	intrinsic_matrix.at<double>(0, 2) = 319.592; //235.48376383;
  	intrinsic_matrix.at<double>(1, 2) = 237.538; //235.36168732;
  	intrinsic_matrix.at<double>(2, 2) = 1.0;
	
	distCoeffs.at<double>(0) = 0.20851136;
	distCoeffs.at<double>(1) = -0.15560189;
	distCoeffs.at<double>(2) = 0.05348765;
	distCoeffs.at<double>(3) = -0.00803635;

	fx = intrinsic_matrix.at<double>(0, 0);
	fy = intrinsic_matrix.at<double>(1, 1);
	cx = intrinsic_matrix.at<double>(0, 2);
	cy = intrinsic_matrix.at<double>(1, 2);

	k1 = 0.20851136;
	k2 = -0.15560189;
	k3 = 0.05348765;
	k4 = -0.00803635;
	k5 = 0; 
	k6 = 0;

}

void CameraModel::projectPoints(const std::vector<cv::Point3d>& points3, 
								const cv::Mat& R, const cv::Mat& T, 
								std::vector<cv::Point2d>& points2)
{
	setTransformation(R, T);
	for(int i = 0; i < points3.size(); i++)
	{
		cv::Point2d pt = getImagePoint(points3[i]);
		points2.push_back(pt);
	}
}
 
void CameraModel::setTransformation(const cv::Mat& R, const cv::Mat& t)
{
	r00 = R.at<double>(0,0);
	r01 = R.at<double>(0,1);
	r02 = R.at<double>(0,2);
	r10 = R.at<double>(1,0);
	r11 = R.at<double>(1,1);
	r12 = R.at<double>(1,2);
	r20 = R.at<double>(2,0);
	r21 = R.at<double>(2,1);
	r22 = R.at<double>(2,2);

	t0 = t.at<double>(0,0);
	t1 = t.at<double>(1,0);
	t2 = t.at<double>(2,0);
}

cv::Point3d CameraModel::rotatePoint(cv::Point3d pt)
{
	double X = pt.x;
	double Y = pt.y;
	double Z = pt.z;

	double Xc = r00*X + r01*Y + r02*Z;
	double Yc = r10*X + r11*Y + r12*Z;
	double Zc = r20*X + r21*Y + r22*Z;

	return cv::Point3d(Xc, Yc, Zc);
}

cv::Point3d CameraModel::translatePoint(cv::Point3d pt)
{		
	return cv::Point3d(pt.x+t0, pt.y+t1, pt.z+t2);
}

cv::Point2d CameraModel::getImagePoint(const cv::Point3d& p)
{
	cv::Point3d rotated_point = rotatePoint(p);
	cv::Point3d translated_point = translatePoint(p);
	cv::Point2d projected_point = pinholeProjection(translated_point);
	cv::Point2d fisheye_distorted_point = fisheyeDistortion(projected_point);
	cv::Point2d pixel_fisheye = getPixelCoordinates(fisheye_distorted_point);
	return pixel_fisheye;
}


/* This method perfoms a pinhole projection given a 3d point and returns
   2d point  */
cv::Point2d CameraModel::pinholeProjection(const cv::Point3d& pt)
{
	double a = pt.x/pt.z;
	double b = pt.y/pt.z;

	//std::cout<<"Point after pinhole projection::\n"<<cv::Point2d(a, b)<<std::endl;

	return cv::Point2d(a, b);
}

/* This method performs fisheye distortion */
cv::Point2d CameraModel::fisheyeDistortion(const cv::Point2d& pt)
{
	double r = std::sqrt(pt.x*pt.x + pt.y*pt.y);

	double theta = std::atan(r);

	//std::cout<<"theta for fisheye distortion: "<<theta<<std::endl;

	double theta_d = theta*(1 + k1*std::pow(theta, 2) + k2*std::pow(theta,4) + k3*std::pow(theta,6) + k4*std::pow(theta,8));

	//std::cout<<"Theta_d for fishete distortion: "<<theta_d<<std::endl;

	double x = (theta_d/r)*pt.x;
	double y = (theta_d/r)*pt.y;

	//std::cout<<"Point after fisheye distortion: "<<cv::Point2d(x, y)<<std::endl;

	return cv::Point2d(x, y);

}

/* This method converts the distorted or pinhole projected points to pixel in the camera coordinate*/
cv::Point2d CameraModel::getPixelCoordinates(const cv::Point2d& pt)
{
	double u = fx*pt.x + cx;
	double v = fy*pt.y + cy;

	//std::cout<<str<<" camera coordinate: "<<cv::Point2d(u, v)<<std::endl;

	return cv::Point2d(u, v);
}