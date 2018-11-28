#include "stdafx.h"
#include "funcStream.h"

void funcStream::depthColorizer(cv::Mat & matOutput, rs2::depth_frame & depth)
{
	rs2::colorizer colorize;
	colorize.set_option(RS2_OPTION_COLOR_SCHEME, 0);
	cv::Mat matDepth = funcFormat::frame2Mat(depth.apply_filter(colorize));
	matOutput = matDepth.clone();
}


rs2::depth_frame funcStream::streamSelector(cv::Mat & matOutput, stream stream, rs2::pipeline & pipeline, rs2::decimation_filter & filterDec, rs2::spatial_filter & filterSpat, rs2::temporal_filter & filterTemp, rs2_stream align, configZoomer & config)
{
	rs2::align alignTo(align);
	rs2::frameset data = pipeline.wait_for_frames();
	rs2::frameset alignedFrame = alignTo.process(data);

	rs2::depth_frame depth = alignedFrame.get_depth_frame();
	//depth = filterDec.process(depth);
	depth = filterSpat.process(depth);
	depth = filterTemp.process(depth);
	
	cv::Mat matFrame, matFrameOrig;
	switch (stream)
	{
	case STREAM_COLOR:
		matFrame = funcFormat::frame2Mat(alignedFrame.get_color_frame());
		matFrameOrig = matFrame.clone();
		streamZoomer(matFrameOrig, matFrame, config.pixelZoom, config.pixelRoiZoom, config.scaleZoom, config.miniMap);
		matOutput = matFrame.clone();
		return depth;
		break;
	case STREAM_INFRARED:
		// =========================================================================
		// align infrared to color is not possible, so call data not call alignedFrame here
		// =========================================================================
		if (align == RS2_STREAM_COLOR)
			matFrame = funcFormat::frame2Mat(data.get_infrared_frame());
		else if (align == RS2_STREAM_DEPTH)
			matFrame = funcFormat::frame2Mat(alignedFrame.get_infrared_frame());
		
		matFrameOrig = matFrame.clone();
		streamZoomer(matFrameOrig, matFrame, config.pixelZoom, config.pixelRoiZoom, config.scaleZoom, config.miniMap);
		matOutput = matFrame.clone();
		return depth;
		break;
	case STREAM_DEPTH:
		funcStream::depthColorizer(matFrame, depth);
		matFrameOrig = matFrame.clone();
		streamZoomer(matFrameOrig, matFrame, config.pixelZoom, config.pixelRoiZoom, config.scaleZoom, config.miniMap);
		matOutput = matFrame.clone();
		return depth;
		break;
	default:
		return 0;
		break;
	}
}

void funcStream::streamInfoer(cv::Mat * input, std::string text)
{
	cv::Size size = input->size();

	cv::putText(*input, text, cv::Point(10, size.height - 10), inforerFontA, 1, inforerColorFA, 1, cv::LINE_AA);
	cv::putText(*input, text, cv::Point(10, size.height - 10), inforerFontB, 1, inforerColorFB, 1, cv::LINE_AA);
}

void funcStream::streamInfoerB(cv::Mat * input, std::string text)
{
	cv::Size size = input->size();
	cv::copyMakeBorder(*input, *input, 0, 40, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	cv::putText(*input, text, cv::Point(10, size.height + 30), inforerFont, 1, inforerColor, 1, cv::LINE_AA);
}

void funcStream::streamMapperRD(cv::Mat & input, cv::Mat & miniInput, cv::Mat & output, cv::Size & sizeMap, int border, cv::Scalar color)
{
	output = input.clone();
	cv::Size size = output.size();
	cv::Mat minimap = miniInput.clone();
	cv::resize(minimap, minimap, sizeMap, 0, 0, CV_INTER_LINEAR);
	cv::copyMakeBorder(minimap, minimap, border, border, border, border, cv::BORDER_CONSTANT, color);
	minimap.copyTo(output(cv::Rect(size.width - minimap.cols - 10, size.height - minimap.rows - 10,
		minimap.cols, minimap.rows)));
}

void funcStream::streamMapperLD(cv::Mat & input, cv::Mat & miniInput, cv::Mat & output, cv::Size & sizeMap, int border, cv::Scalar color)
{
	output = input.clone();
	cv::Size size = output.size();
	cv::Mat minimap = miniInput.clone();
	cv::resize(minimap, minimap, sizeMap, 0, 0, CV_INTER_LINEAR);
	cv::copyMakeBorder(minimap, minimap, border, border, border, border, cv::BORDER_CONSTANT, color);
	minimap.copyTo(input(cv::Rect(10, size.height - minimap.rows - 10,
		minimap.cols, minimap.rows)));
}

void funcStream::streamZoomer(cv::Mat & input, cv::Mat & output, cv::Point & pixelZoom, cv::Point & pixelRoiZoom, float & scaleZoom, bool mini)
{
	if (scaleZoom == 1)
	{
		output = input.clone();
	}
	else
	{
		pixelZoom.x = (int)(pixelZoom.x * scaleZoom) + pixelRoiZoom.x;
		pixelZoom.y = (int)(pixelZoom.y * scaleZoom) + pixelRoiZoom.y;

		cv::Size size = input.size();
		int scaledWidth = (int)(size.width * scaleZoom);
		int scaledHeight = (int)(size.height * scaleZoom);
		pixelRoiZoom.x = pixelZoom.x - (scaledWidth / 2);
		pixelRoiZoom.y = pixelZoom.y - (scaledHeight / 2);

		if ((pixelZoom.x + (scaledWidth / 2)) > size.width)
			pixelRoiZoom.x = size.width - scaledWidth;

		if ((pixelZoom.y + (scaledHeight / 2)) > size.height)
			pixelRoiZoom.y = size.height - scaledHeight;

		if ((pixelZoom.x - (scaledWidth / 2)) < 0)
			pixelRoiZoom.x = 0;

		if ((pixelZoom.y - (scaledHeight / 2)) < 0)
			pixelRoiZoom.y = 0;

		cv::Rect roi = cv::Rect(pixelRoiZoom.x, pixelRoiZoom.y, scaledWidth, scaledHeight);
		output = input(roi);
		cv::resize(output, output, size, 0, 0, CV_INTER_AREA);

		// generate mini-map
		if (mini)
		{
			cv::Mat outMap = input.clone();
			cv::rectangle(outMap, roi, zoomerLineColor, zoomerLineSize);
			cv::Size sizeMap = cv::Size((int)(size.width / 8), (int)(size.height / 8));
			streamMapperLD(output, outMap, output, sizeMap, zoomerMapSize, zoomerMapColor);
		}
	}
}

void funcStream::streamZoomPixelTrans(cv::Point & input, cv::Point & output, configZoomer & configZoomer)
{
	if (configZoomer.scaleZoom == 1)
	{
		output = input;
	}
	else
	{
		output.x = (int)(input.x * configZoomer.scaleZoom + configZoomer.pixelRoiZoom.x);
		output.y = (int)(input.y * configZoomer.scaleZoom + configZoomer.pixelRoiZoom.y);
	}
}
