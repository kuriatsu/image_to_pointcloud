#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>
#include <image_to_pointcloud/image_to_pointcloudConfig.h>

#include "geometry_msgs/Vector3.h"

struct image_pose
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};


class ImageToPointcloud
{
private:
    ros::Publisher pub_image_cloud;

    // ros::Timer timer;

    dynamic_reconfigure::Server<image_to_pointcloud::image_to_pointcloudConfig> ddynamic_server;
    dynamic_reconfigure::Server<image_to_pointcloud::image_to_pointcloudConfig>::CallbackType ddynamic_server_callback;

    std::string m_image_name;
    cv::Mat m_cv_image;
    image_pose m_image_pose;
    float m_image_res;
    bool m_synchronize = false;

public:
    ImageToPointcloud();
    ~ImageToPointcloud();

private:
    void ddynamicCb(image_to_pointcloud::image_to_pointcloudConfig &config, uint32_t level);
    void makeImageCloud();
    // void timerCb(const ros::TimerEVent&);
    void ddynamicUpdate();
    void ddynamicInit();
};


ImageToPointcloud::ImageToPointcloud()
{
    ros::NodeHandle n;

    ddynamic_server_callback = boost::bind(&ImageToPointcloud::ddynamicCb, this, _1, _2);
    ddynamic_server.setCallback(ddynamic_server_callback);

    pub_image_cloud = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/image_cloud", 5);
}

ImageToPointcloud::~ImageToPointcloud()
{
    ddynamicInit();
}

void ImageToPointcloud::ddynamicCb(image_to_pointcloud::image_to_pointcloudConfig &config, uint32_t level)
{
    std::string image = config.image;
    m_image_res = std::stof(config.image_res);
    m_synchronize = config.synchronize;

    m_image_pose.x = std::stof(config.image_x);
    m_image_pose.y = std::stof(config.image_y);
    m_image_pose.z = std::stof(config.image_z);
    m_image_pose.roll = std::stof(config.image_roll) * M_PI / 180.0;
    m_image_pose.pitch = std::stof(config.image_pitch) * M_PI / 180.0;
    m_image_pose.yaw = std::stof(config.image_yaw) * M_PI / 180.0;


    if (image != m_image_name)
    {
        m_cv_image = cv::imread(image, 1);
        m_image_name = image;
    }

    std::cout << m_synchronize << std::endl;
    if (m_synchronize)
    {
        makeImageCloud();
    }
}


void ImageToPointcloud::ddynamicUpdate()
{
    image_to_pointcloud::image_to_pointcloudConfig config;
    config.image_res = std::to_string(m_image_res);
    config.image_x = std::to_string(m_image_pose.x);
    config.image_y = std::to_string(m_image_pose.y);
    config.image_z = std::to_string(m_image_pose.z);
    config.image_roll = std::to_string(m_image_pose.roll);
    config.image_pitch = std::to_string(m_image_pose.pitch);
    config.image_yaw = std::to_string(m_image_pose.yaw);
    config.synchronize = m_synchronize;
    // config.publish = false;

    ddynamic_server.updateConfig(config);
}


void ImageToPointcloud::ddynamicInit()
{
    image_to_pointcloud::image_to_pointcloudConfig config;
    config.image_res = std::to_string(0.0);
    config.image_x = std::to_string(0.0);
    config.image_y = std::to_string(0.0);
    config.image_z = std::to_string(0.0);
    config.image_roll = std::to_string(0.0);
    config.image_pitch = std::to_string(0.0);
    config.image_yaw = std::to_string(0.0);
    config.synchronize = false;

    ddynamic_server.updateConfig(config);
}

void ImageToPointcloud::makeImageCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB> image_cloud;
    int height = m_cv_image.rows;
    int width = m_cv_image.cols;
    std::cout << width << std::endl;
    for (int i = 0; i < height; i++)
    {
        cv::Vec3b *col = m_cv_image.ptr<cv::Vec3b>(i);
        for (int j=0; j<width; j++)
        {
            pcl::PointXYZRGB point;
            float x = i * m_image_res;
            float y = j * m_image_res;
            float z = 0.0;

            float rotated_x = x * (cos(m_image_pose.yaw) * cos(m_image_pose.pitch))
                      + y * (cos(m_image_pose.yaw) * sin(m_image_pose.pitch) * sin(m_image_pose.roll)
                             - sin(m_image_pose.yaw) * cos(m_image_pose.roll))
                      + z * (cos(m_image_pose.yaw) * sin(m_image_pose.pitch) * cos(m_image_pose.roll)
                             + sin(m_image_pose.yaw) * sin(m_image_pose.roll));
            float rotated_y = x * (sin(m_image_pose.yaw) * cos(m_image_pose.pitch))
                      + y * (sin(m_image_pose.yaw) * sin(m_image_pose.pitch) * sin(m_image_pose.roll)
                             + cos(m_image_pose.yaw) * cos(m_image_pose.roll))
                      + z * (sin(m_image_pose.yaw) * sin(m_image_pose.pitch) * cos(m_image_pose.roll)
                             - cos(m_image_pose.yaw) * sin(m_image_pose.roll));
            float rotated_z = x * (-sin(m_image_pose.pitch))
                      + y * (cos(m_image_pose.pitch) * sin(m_image_pose.roll))
                      + z * (cos(m_image_pose.pitch) * cos(m_image_pose.roll));

            point.x = rotated_x + m_image_pose.x;
            point.y = rotated_y + m_image_pose.y;
            point.z = rotated_z + m_image_pose.z;
            point.r = col[j][2];
            point.g = col[j][1];
            point.b = col[j][0];
            // std::cout << point << std::endl;
            image_cloud.push_back(point);
        }
    }
    std::cout << "pub" << std::endl;

    auto msg = image_cloud.makeShared();
    msg->header.frame_id = "map";
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

    pub_image_cloud.publish(msg);
    ddynamicUpdate();
}

//
// void AdasVis::timerCallback(const ros::TimerEvent&)
// {
//     if (m_mode)
//     {
//         makeImageCloud();
//     }
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_pointcloud_node");

    ImageToPointcloud image_to_pointcloud;

    ros::spin();

    return 0;
}
