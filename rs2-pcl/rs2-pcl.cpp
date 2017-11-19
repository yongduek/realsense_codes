
#include <iostream>
#include <fstream>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

using namespace std;


void display_depth (cv::Mat_<float> depth)
{
	cv::Mat_<float> d = depth.clone();

	double min, max;
	cv::minMaxLoc (d, &min, &max);

	d /= max; // scale d so that its pixel range is 0 to 1

	cv::imshow ("depth image", d);
}


static std::string get_device_name(const rs2::device& dev)
{
	// Each device provides some information on itself, such as name:
	std::string name = "Unknown Device";
	if (dev.supports(RS2_CAMERA_INFO_NAME))
		name = dev.get_info(RS2_CAMERA_INFO_NAME);

	// and the serial number of the device:
	std::string sn = "########";
	if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
		sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

	return name + " " + sn;
}

static void print_device_info(const rs2::device& dev)
{
	std::cout << "Device information: " << std::endl;
	//The following code shows how to enumerate all of the RS2_CAMERA_INFO
	//Note that all enum types in the SDK start with the value of zero and end at the "*_COUNT" value
	for (int i = 0; i < static_cast<int>(RS2_CAMERA_INFO_COUNT); i++)
		{
			rs2_camera_info info_type = static_cast<rs2_camera_info>(i);
			//SDK enum types can be streamed to get a string that represents them
			std::cout << "  " << std::left << std::setw(20) << info_type << " : ";

			//A device might not support all types of RS2_CAMERA_INFO.
			//To prevent throwing exceptions from the "get_info" method we first check if the device supports this type of info
			if (dev.supports(info_type))
				std::cout << dev.get_info(info_type) << std::endl;
			else
				std::cout << "N/A" << std::endl;
		}
}

static std::string get_sensor_name(const rs2::sensor& sensor)
{
	// Sensors support additional information, such as a human readable name
	if (sensor.supports(RS2_CAMERA_INFO_NAME))
		return sensor.get_info(RS2_CAMERA_INFO_NAME);
	else
		return "Unknown Sensor";
}

static void print_intrinsics(const rs2_intrinsics& intrinsics, string head="")
{
	if (!head.empty()) {
		cerr << head << endl;
	}
	auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
	auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
	rs2_distortion model = intrinsics.model;

	std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
	std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
	std::cout << "Distortion Model        : " << model << std::endl;
	std::cout << "Distortion Coefficients : [" << intrinsics.coeffs[0] << "," << intrinsics.coeffs[1] << ","
			  << intrinsics.coeffs[2] << "," << intrinsics.coeffs[3] << "," << intrinsics.coeffs[4] << "]" << std::endl;
}

static rs2_intrinsics  get_field_of_view(const rs2::stream_profile& stream)
{
	rs2_intrinsics intrinsics;
	// A sensor's stream (rs2::stream_profile) is in general a stream of data with no specific type.
	// For video streams (streams of images), the sensor that produces the data has a lens and thus
	// has properties such as a focal point, distortion, and principal point.
	// To get these intrinsics parameters,
	//  we need to take a stream and first check if it is a video stream
	if (auto video_stream = stream.as<rs2::video_stream_profile>()) {
		try
			{
				//If the stream is indeed a video stream, we can now simply call get_intrinsics()
				intrinsics = video_stream.get_intrinsics();
			}
		catch (const std::exception& e)
			{
				std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
			}
	}
	else {
		std::cerr << "Given stream profile is not a video stream profile" << std::endl;
	}

	return intrinsics;
}

struct RS2Data
{
	// Constructor
	RS2Data() {};

	// initialize
	// 1. initialize pipeline
	// 2. retrieve (depth intrinsics, depth_units)
	void start ()
	{
		pipe.start();
		for (auto i=0; i<30; i++) pipe.wait_for_frames();

        const rs2::pipeline_profile pp = pipe.get_active_profile();

		// current stream info
		const std::vector<rs2::stream_profile> strm_profs = pp.get_streams();
		int index=0;
		for (rs2::stream_profile sp : strm_profs)
			{
				std::cerr << "@ stream " << index++ << " intrinsics:" << std::endl;
				rs2_intrinsics intrinsics = get_field_of_view (sp);
				//print_intrinsics(intrinsics, string("@ intrinsic of ") + to_string(index));
			}
		// 0-th sensor is the depth sensor
		this->intrinsics = strm_profs[0].as<rs2::video_stream_profile>().get_intrinsics(); 
		print_intrinsics (this->intrinsics, "@ retrieved intrinsics ---- ");
		
		double scale = pp.get_device().query_sensors()[0].as<rs2::depth_sensor>().get_depth_scale();
		cerr << "-- scale = " << scale << endl;
		this->depth_units = scale;
	  
		// device info from pipeline_profile
		// depth_units can also be retrieved from the following.
		if (0)
		{
			const rs2::device dev = pp.get_device();
			print_device_info(dev);

			std::vector<rs2::sensor> sensors = dev.query_sensors();
			std::cout << "@ Device consists of " << sensors.size() << " sensors:\n" << std::endl;
			int index = 0;
			// We can now iterate the sensors and print their names
			for (rs2::sensor sensor : sensors)
				{
					std::cout << "  " << index++ << " : " << get_sensor_name(sensor) << std::endl;
					if (rs2::depth_sensor dpt_sensor = sensor.as<rs2::depth_sensor>())
						{
							this->depth_units = dpt_sensor.get_depth_scale();
							std::cout << "@@ Scale factor for depth sensor: " << depth_units << std::endl;
							//std::vector<rs2::stream_profile> vec_sp = sensor.get_stream_profiles();
							//std::cerr << "@@ stream_profile size = " << vec_sp.size() << std::endl;
						}
				}
		}

	}

	void grab(pcl::PointCloud<pcl::PointXYZ>* pCloud=0)
	{
		frameset = pipe.wait_for_frames();
		rs2::depth_frame depthf = frameset.get_depth_frame();
		rs2::video_frame colorf   = frameset.get_color_frame();
		points = pc.calculate (depthf);
		int height = depthf.get_height(), width = depthf.get_width();

        {
            //auto depth_frame = (rs2::frame_interface*) depthf.get();
            //double depth_unit = depth_frame->get_sensor()->get_option(RS_OPTION_DEPTH_UNITS).query();
            //cerr << "@ depth_unit = " << depth_unit << endl;
        }

		// retrieve vertex point cloud
		if (pCloud)
			{
				const rs2::vertex* vertex= points.get_vertices();
				cerr << "@ size of pcd = " << points.size() << endl;

				for (auto i=0; i<points.size(); i++) {
					int r = i / width;
					int c = i % width;
					pCloud->points[i].x = vertex[i].x;
					pCloud->points[i].y = vertex[i].y;
					pCloud->points[i].z = vertex[i].z;
				}
			}
		// retrieve depth image
		//


	 
		if (0)
		{
			cv::Mat_<float> dimg(depthf.get_height(), depthf.get_width());
			cerr << "@ depth frame: " << dimg.rows << "x" << dimg.cols << endl;
			
			const rs2::vertex* vertex= points.get_vertices();
			cerr << "@ size of pcd = " << points.size() << endl;

			float maxz = 0;
			for (auto i=0; i<points.size(); i++) {
				int r = i / dimg.cols;
				int c = i % dimg.cols;
				dimg(r,c) = vertex[i].z;
				if (maxz < vertex[i].z) maxz = vertex[i].z;
			}

			cerr << "@ max depth = " << maxz << endl;

			display_depth (dimg);
			cv::waitKey(10);
		}		
	}

public:
    rs2::device   device;
	rs2::pipeline pipe;
	rs2::points   points;
	rs2::pointcloud pc;
	
	rs2::frameset frameset;

	rs2_intrinsics intrinsics;
	double depth_units ;
};


int main (int argc, char** argv)
{
	int nFrames = 2;

    std::vector<cv::Mat> vrgb;
	std::vector<cv::Mat> vdepth; // 1 channel depth image

	RS2Data rs2data;
	rs2data.start();

	pcl::PointCloud<pcl::PointXYZ> cloud;
	// Fill in the cloud data
	cloud.width    = 640;
	cloud.height   = 480;
	cloud.is_dense = true;
	cloud.points.resize (cloud.width * cloud.height);

	for (auto i=0; i<nFrames; i++) {
		rs2data.grab(&cloud);
		char filename[512];
		sprintf (filename, "test-pcd-%0d.pcd", i);
		pcl::io::savePCDFileASCII (filename, cloud);
	}


  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  return 0;
  
  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
