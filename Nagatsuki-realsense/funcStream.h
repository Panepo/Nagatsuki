#pragma once
#ifndef FUNCSTREAM_H
#define FUNCSTREAM_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include "funcFormat.h"

#define inforerFontA	cv::FONT_HERSHEY_DUPLEX
#define inforerFontB	cv::FONT_HERSHEY_SIMPLEX
#define inforerColorFA	cv::Scalar(0, 0, 0)			// black
#define inforerColorFB	cv::Scalar(0, 255,0)		// green

#define inforerFont		cv::FONT_HERSHEY_TRIPLEX
#define inforerColor	cv::Scalar(0, 255,0)		// green

#define zoomerScaleMin	0.1
#define zoomerScaleMax	1
#define zoomerLineSize	5
#define zoomerLineColor cv::Scalar(0, 255,255)		// yellow
#define zoomerMapSize	3
#define zoomerMapColor  cv::Scalar(0, 255,0)		// green

typedef enum stream
{
	STREAM_COLOR,
	STREAM_INFRARED,
	STREAM_DEPTH,
	STREAM_FILE,
} stream;

class configZoomer
{
public:
	cv::Point pixelZoom = cv::Point(0, 0);
	cv::Point pixelRoiZoom = cv::Point(0, 0);
	float scaleZoom = 1;
	bool miniMap = true;
};

namespace funcStream
{
	rs2::depth_frame streamSelector(
		cv::Mat & matOutput,
		stream stream,
		rs2::pipeline & pipeline,
		rs2::decimation_filter & filterDec,
		rs2::spatial_filter & filterSpat,
		rs2::temporal_filter & filterTemp,
		rs2_stream align,
		configZoomer & config
	);
	
	void depthColorizer(cv::Mat & matOutput, rs2::depth_frame & depth);
	
	void streamInfoer(cv::Mat* input, std::string text);
	void streamInfoerB(cv::Mat* input, std::string text);
	
	void streamMapperRD(cv::Mat & input, cv::Mat & miniInput, cv::Mat & output, cv::Size & sizeMap, int border, cv::Scalar color);
	void streamMapperLD(cv::Mat & input, cv::Mat & miniInput, cv::Mat & output, cv::Size & sizeMap, int border, cv::Scalar color);
	
	void streamZoomer(cv::Mat & input, cv::Mat & output, cv::Point & pixelZoom, cv::Point & pixelRoiZoom, float & scaleZoom, bool mini);

	void streamZoomPixelTrans(cv::Point & input, cv::Point & output, configZoomer & configZoomer);
}


#endif