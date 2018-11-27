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
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	cv::namedWindow("Road facing camera", cv::WINDOW_AUTOSIZE);

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(pathORBvoc, pathCamera, ORB_SLAM2::System::MONOCULAR, true);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	cv::Mat frame, input, inputGray;
	bool slam = false;

	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point t2 = t1;

	for (;;)
	{
		cap >> frame;
		if (frame.empty())
			break;

		frame.copyTo(input);
		t1 = std::chrono::steady_clock::now();

		if (slam)
		{
			cv::cvtColor(input, inputGray, cv::COLOR_BGR2GRAY);
			double tframe = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t2).count();
			SLAM.TrackMonocular(inputGray, tframe);
		}

		t2 = t1;
		cv::imshow("Road facing camera", input);

		char c = (char)cv::waitKey(10);
		if (c == 27)
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

