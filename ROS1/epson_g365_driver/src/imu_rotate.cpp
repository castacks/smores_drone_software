#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>

ros::Publisher imu_pub;

double yaw = M_PI/2;
double pitch = 0.0;
double roll = M_PI;

Eigen::Quaterniond rot;

Eigen::Quaterniond ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(-pitch * 0.5);
    double sp = sin(-pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    double cc = cy*cr;
    double cs = cy*sr;
    double sc = sy*cr;
    double ss = sy*sr;

    
    double w =  cp * cc + sp * ss;
    double y = -(cp * ss + sp * cc);
    double x =  cp * cs - sp * sc;
    double z =  cp * sc - sp * cs;


    Eigen::Quaterniond q(w,x,y,z);
    if (w < 0)
        q = Eigen::Quaterniond(-w, -x, -y, -z);
    printf("rpy: %f %f %f, q(w,x,y,z): %f %f %f %f]\n", roll, pitch, yaw, q.w(), q.x(), q.y(), q.z());

    return q;
}



void imu_callback(const sensor_msgs::ImuConstPtr imu_in)
{
    Eigen::Vector3d acc;
    acc << imu_in->linear_acceleration.x,
        imu_in->linear_acceleration.y,
        imu_in->linear_acceleration.z;
    Eigen::Vector3d gyr;
    gyr << imu_in->angular_velocity.x,
        imu_in->angular_velocity.y,
        imu_in->angular_velocity.z;

    Eigen::Quaterniond orientation(imu_in->orientation.w,
                           imu_in->orientation.x,
                           imu_in->orientation.y, 
                           imu_in->orientation.z);

    Eigen::Vector3d rot_acc = rot * acc;
    Eigen::Vector3d rot_gyr = rot * gyr;
    Eigen::Quaterniond rot_orientation = rot * orientation;

    sensor_msgs::Imu imu_out = *imu_in;

    imu_out.linear_acceleration.x = rot_acc.x();
    imu_out.linear_acceleration.y = rot_acc.y();
    imu_out.linear_acceleration.z = rot_acc.z();

    imu_out.angular_velocity.x = rot_gyr.x();
    imu_out.angular_velocity.y = rot_gyr.y();
    imu_out.angular_velocity.z = rot_gyr.z();

    imu_out.orientation.w = rot_orientation.w();
    imu_out.orientation.x = rot_orientation.x();
    imu_out.orientation.y = rot_orientation.y();
    imu_out.orientation.z = rot_orientation.z();


    imu_pub.publish(imu_out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_rotate");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    // Subscriber

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("epson_imu", 100, &imu_callback);

    imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 100);

    rot = ToQuaternion(yaw, pitch, roll);

    ros::spin();

}
