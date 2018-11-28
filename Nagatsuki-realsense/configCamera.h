#pragma once
#ifndef CONFIGCAMERA_H
#define CONFIGCAMERA_H

#include <librealsense2/rs.hpp>

typedef enum supportDevice
{
	REALSENSE_415,
	REALSENSE_435,
} supportDevice;

static supportDevice detectDevice()
{
	rs2::context ctx = rs2::context();
	rs2::device_list devices = ctx.query_devices();
	rs2::device selected_device;

	if (devices.size() == 0)
	{
		throw std::runtime_error("No device connected, please connect a RealSense 415 or 435.");
		//rs2::device_hub device_hub(ctx);
		//selected_device = device_hub.wait_for_device();
	}
	else
	{
		std::string name = "Unknown Device";
		
		for (rs2::device device : devices)
		{
			if (device.supports(RS2_CAMERA_INFO_NAME))
				name = device.get_info(RS2_CAMERA_INFO_NAME);
			
			std::cout << "Detected device: " << name << std::endl;

			if (name == "Intel RealSense D415")
				return REALSENSE_415;
			else if (name == "Intel RealSense D435")
				return REALSENSE_435;
		}
		throw std::runtime_error("No device connected, please connect a RealSense 415 or 435.");
	}
}

#endif // !CONFIGCAMERA_H
