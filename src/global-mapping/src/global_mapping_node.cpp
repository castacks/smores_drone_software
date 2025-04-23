//
// Created by smores on 4/21/25.
//
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <pcl_ros/transforms.hpp>
using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;


class GlobalMappingNode : public rclcpp::Node {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_points_subscriber;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;

public:
    GlobalMappingNode()
        : Node("global_mapping") {
        rclcpp::QoS qos = rclcpp::QoS(10);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
        depth_points_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/foundation_points", 10, std::bind(&GlobalMappingNode::pointsCallback, this, std::placeholders::_1));
        map_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        global_map =  pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

    void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received Points!! in frame: "<<msg->header.frame_id);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform("map", msg->header.frame_id, msg->header.stamp);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_ros::transformPointCloud(*cloud, *transformed_cloud, transform_stamped);

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(transformed_cloud);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // Adjust leaf size as needed
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.filter(*downsampled_cloud);

        *global_map += *downsampled_cloud;

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_map;
        voxel_filter_map.setInputCloud(global_map);
        voxel_filter_map.setLeafSize(0.1f, 0.1f, 0.1f); // Adjust leaf size as needed
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_map(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter_map.filter(*downsampled_map);

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*downsampled_map, output_msg);
        output_msg.header.frame_id = "map";
        output_msg.header.stamp = msg->header.stamp;
        RCLCPP_INFO_STREAM(this->get_logger(), "PUBLISHING");
        RCLCPP_INFO(this->get_logger(), "Downsampled cloud has %zu points", downsampled_cloud->size());
        RCLCPP_INFO(this->get_logger(), "Global map has %zu points", global_map->size());
        map_publisher->publish(output_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalMappingNode>());
    rclcpp::shutdown();
    return 0;
}
