
#include <iostream>
#include <fstream>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

using namespace std;

void rs2_deproject_pixel_to_point(float point[3], 
                const struct rs2_intrinsics * intrin, const float pixel[2], const float depth)
{
    assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
    assert(intrin->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
    //assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

    float x = (pixel[0] - intrin->ppx) / intrin->fx;
    float y = (pixel[1] - intrin->ppy) / intrin->fy;
    if(intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
    {
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        float ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }
    point[0] = depth * x;
    point[1] = depth * y;
    point[2] = depth;
}

cv::Mat_<float> get_depth_frame (const rs2::depth_frame& depth, const float depth_unit)
{
        cv::Mat_<cv::Vec3f> dimg(depth.get_height(), depth.get_width());

        auto depth_data = (const uint16_t*) depth.get_data();

        for (auto i=0; i<dimg.rows; i++) {
                for (auto j=0; j<dimg.cols; j++) {
                        auto k = i*dimg.cols + j;
                        float depth = depth_data[k] * depth_unit;
                        float p3[3], p2[2];
                        rs2_intrinsics *depth_intrinsics;
                        rs2_deproject_pixel_to_point (p3, depth_intrinsics, p2, depth);
                }
        }

        return dimg;
}

struct rs2data {
};

int main (int argc, char** argv)
{
        int nFrames = 2;

        rs2::pipeline pipe;
        pipe.start();
        for (auto i=0; i<30; i++) pipe.wait_for_frames();

        std::vector<cv::Mat> vrgb;
        std::vector<cv::Mat> vdepth; // 1 channel depth image

        for (auto i=0; i<nFrames; i++) {
            auto frames = pipe.wait_for_frames();
            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();

            float depth_unit = 0.1; // retrieve from the sensor
            cv::Mat_<cv::Vec3f> dimg = get_depth_frame (depth, depth_unit);
        }

  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
