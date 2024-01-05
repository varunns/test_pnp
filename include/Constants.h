#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <filesystem>
#include <string>
#include <fstream>

class Constants
{
private:
	const std::vector<cv::Point3d> led_points_3d {
												cv::Point3d(0.00511406,0.0565338,0.0140667),
												cv::Point3d(-0.011315,0.0363465,0.0135623),
												cv::Point3d(-0.0201706,0.0371088,0.0396103),
												cv::Point3d(-0.0210794,0.014144,0.0411221),
												cv::Point3d(-0.0154443,0.019355,0.0631506),
												cv::Point3d(-0.00636688,-0.00466865,0.0645783),
												cv::Point3d(0.0114907,0.00722219,0.0801598),
												cv::Point3d(0.0380356,-0.0094137,0.0706843),
												cv::Point3d(0.0465194,0.0112052,0.0746705),
												cv::Point3d(0.0582015,0.00101502,0.0581461),
												cv::Point3d(0.0621919,0.0242527,0.0577729),
												cv::Point3d(0.0652631,0.0175827,0.0365578),
												cv::Point3d(0.0620529,0.0416927,0.033129),
												cv::Point3d(0.052483,0.0381296,0.0105479),
												cv::Point3d(0.0406166,0.0558883,0.0141992) };


	std::vector<std::vector<int> > combinations_3d;

	void readCombinationFile(const std::string& str)
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
			combinations_3d.push_back(row);
		}

	}
public:
	std::vector<cv::Point3d> getPoints3d()
	{
		return led_points_3d;
	}

	std::vector<std::vector<int> > getCombinations()
	{
		return combinations_3d;
	}


	Constants()
	{
		//readCombinationFile(file_path);
	}
};

#endif