#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

std::vector<size_t> px_offset_front{37, 25, 13, 0, 37, 25, 13, 1, 37, 25, 13, 1, 37, 25, 13, 1, 37, 25, 13, 1, 37, 25, 13, 1, 37, 25, 13, 1, 37, 25, 14, 2, 37, 25, 14, 2, 37, 26, 14, 2, 37, 26, 14, 2, 38, 26, 14, 2, 38, 26, 14, 2, 38, 26, 14, 2, 38, 26, 14, 2, 39, 26, 14, 2};
std::vector<size_t> px_offset_port{25, 0, 25, 1, 25, 1, 25, 1, 25, 1, 25, 1, 25, 1, 25, 2, 26, 2, 26, 2, 26, 2, 26, 2, 26, 2, 26, 2, 26, 2, 26, 2};
std::vector<size_t> px_offset_starboard{24, 0, 25, 1, 25, 1, 25, 1, 25, 1, 25, 2, 25, 2, 25, 2, 25, 2, 26, 2, 26, 2, 26, 2, 26, 2, 26, 2, 27, 2, 27, 2};

struct PointXYZIR {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;
    std::uint8_t ring;
    std::uint16_t reflectivity;
    std::uint16_t ambient;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint8_t, ring, ring) (std::uint32_t, t, t) (std::uint16_t, reflectivity, reflectivity) 
    (std::uint16_t, ambient, ambient) (std::uint32_t, range, range)
)

template<typename T>
cv::Mat convertToImg(pcl::PointCloud<T> & pohang_cloud, std::vector<size_t> & offset, size_t height, size_t width, std::string image_type) {
    cv::Mat image(height, width, CV_8UC1, cv::Scalar(0));

    for (size_t u = 0; u < height; ++u) {
        for (size_t v = 0; v < width; ++v) {
            size_t vv;
            vv = (v + width - offset[u]) % width;
            const size_t index = u * width + vv;
            const auto& pt = pohang_cloud[index];

            if(image_type == "intensity") {
                int intensity = int(pt.intensity);
                image.at<uint8_t>(u, v) = std::min(intensity, 255);                
            } else if (image_type == "ambient") {
                int ambient = int(pt.ambient/8);
                image.at<uint8_t>(u, v) = std::min(ambient, 255);
            } else {
                return image;
            }

        }
    }

    return image;
}

void OnSubscribeFrontPointCloud(const sensor_msgs::PointCloud2ConstPtr & msg) {
    pcl::PointCloud<PointXYZIR> point_cloud;
    pcl::fromROSMsg(*msg, point_cloud);

    cv::Mat img = convertToImg(point_cloud, px_offset_front, 64, 2048, "ambient");

    cv::resize(img, img, cv::Size(img.cols, img.rows*3));
    cv::imshow("Front Ambient Image", img);
    cv::waitKey(1);
}

void OnSubscribePortPointCloud(const sensor_msgs::PointCloud2ConstPtr & msg) {
    pcl::PointCloud<PointXYZIR> point_cloud;
    pcl::fromROSMsg(*msg, point_cloud);

    cv::Mat img = convertToImg(point_cloud, px_offset_port, 32, 2048, "ambient");

    cv::resize(img, img, cv::Size(img.cols, img.rows*6));
    cv::imshow("Port Ambient Image", img);
    cv::waitKey(1);
}

void OnSubscribeStarboardPointCloud(const sensor_msgs::PointCloud2ConstPtr & msg) {
    pcl::PointCloud<PointXYZIR> point_cloud;
    pcl::fromROSMsg(*msg, point_cloud);

    cv::Mat img = convertToImg(point_cloud, px_offset_starboard, 32, 2048, "ambient");

    cv::resize(img, img, cv::Size(img.cols, img.rows*6));
    cv::imshow("Starboard Ambient Image", img);
    cv::waitKey(1);
}

int main(int argc, char ** argv) {
    printf("LiDAR to Image Node\n");

    ros::init(argc, argv, "lidar_to_iamge_node");
    ros::NodeHandle nh;

    ros::Subscriber subLFrontiDAR = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_front/os_cloud_node/points", 1, OnSubscribeFrontPointCloud);
    ros::Subscriber subPortLiDAR = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_port/os_cloud_node/points", 1, OnSubscribePortPointCloud);
    ros::Subscriber subStarboardLiDAR = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_starboard/os_cloud_node/points", 1, OnSubscribeStarboardPointCloud);

    ros::spin();

    return 0;
}