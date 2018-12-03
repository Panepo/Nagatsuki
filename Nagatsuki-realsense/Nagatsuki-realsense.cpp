// Nagatsuki-realsense.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2\opencv.hpp>
#include <librealsense2/rs.hpp>
#include "System.h"
#include "configCamera.h"
#include "funcStream.h"

int main(int argc, char * argv[]) try
{
	// RealSense settings
	rs2::pipeline pipeline;
	rs2::config config;
	config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	rs2::pipeline_profile cfg = pipeline.start(config);

	// ORB SLAM settings
	ORB_SLAM2::System SLAM("ORBvoc.txt", "realsense.yaml", ORB_SLAM2::System::MONOCULAR, true);
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
		rs2::frameset data = pipeline.wait_for_frames();
		frame = funcFormat::frame2Mat(data.get_color_frame());
		input = frame.clone();

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
		cv::putText(input, str, cv::Point(10, size.height - 10), inforerFontA, 1, inforerColorFA, 1, cv::LINE_AA);
		cv::putText(input, str, cv::Point(10, size.height - 10), inforerFontB, 1, inforerColorFB, 1, cv::LINE_AA);

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

	pipeline.stop();
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	system("pause");
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	system("pause");
	return EXIT_FAILURE;
}

