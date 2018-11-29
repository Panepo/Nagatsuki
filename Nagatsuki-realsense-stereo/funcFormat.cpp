#include "stdafx.h"
#include "funcFormat.h"

namespace funcFormat
{
	cv::Mat frame2Mat(const rs2::frame & f)
	{
		auto vf = f.as<rs2::video_frame>();
		const int w = vf.get_width();
		const int h = vf.get_height();

		if (f.get_profile().format() == RS2_FORMAT_BGR8)
		{
			return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
		}
		else if (f.get_profile().format() == RS2_FORMAT_RGB8)
		{
			auto r = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
			cv::cvtColor(r, r, CV_BGR2RGB);
			return r;
		}
		else if (f.get_profile().format() == RS2_FORMAT_Z16)
		{
			return cv::Mat(cv::Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
		}
		else if (f.get_profile().format() == RS2_FORMAT_Y8)
		{
			return cv::Mat(cv::Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);;
		}

		throw std::runtime_error("Frame format is not supported yet!");
	}
}