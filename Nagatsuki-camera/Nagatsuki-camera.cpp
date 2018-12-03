// Nagatsuki-camera.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2\opencv.hpp>

#include "System.h"

const std::string pathORBvoc = "ORBvoc.txt";
const std::string pathCamera = "FN80AF.yaml";

int main(int argc, char * argv[]) try
{
	// OpenCV camera settings
	cv::VideoCapture cap;
	cap.open(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	cv::namedWindow("Road facing camera", cv::WINDOW_AUTOSIZE);

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(pathORBvoc, pathCamera, ORB_SLAM2::System::MONOCULAR, true);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	cv::Mat frame, input, inputGray;
	bool slam = false;
	double tframe;

	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point t2 = t1;
	typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;

	for (;;)
	{
		cap >> frame;
		if (frame.empty())
			break;

		frame.copyTo(input);
	
		t1 = std::chrono::steady_clock::now();
		tframe = std::chrono::duration_cast<ms>(t1 - t2).count();
		if (slam)
		{
			cv::cvtColor(input, inputGray, cv::COLOR_BGR2GRAY);
			SLAM.TrackMonocular(inputGray, tframe);
		}
		t2 = t1;
		std::ostringstream strs;
		strs << tframe;
		std::string str = strs.str() + " ms";

		cv::Size size = input.size();
		cv::putText(input, str, cv::Point(10, size.height - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
		cv::putText(input, str, cv::Point(10, size.height - 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
		cv::imshow("Road facing camera", input);

		char c = (char)cv::waitKey(10);
		if (c == 27 || c == 81 || c == 213)
			break;
		switch (c)
		{
		case 'a':
			slam = !slam;
			if (slam)
			{
				cout << endl << "-------" << endl;
				cout << "Start ORB-SLAM" << endl;
			}
			else
			{
				cout << endl << "-------" << endl;
				cout << "Stop ORB-SLAM" << endl;
			}
			break;
		default:
			;
		}
	}
	
	// Stop all threads
	SLAM.Shutdown();

	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
	
	cap.release();
	return EXIT_SUCCESS;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	system("pause");
	return EXIT_FAILURE;
}

