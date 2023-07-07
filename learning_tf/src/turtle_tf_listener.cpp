#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "ycao_tf_listener");

    ros::NodeHandle nh;

    ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("/spawn");

    ros::service::waitForService("/spawn");
    
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;
    bool ok = spawnClient.call(req, resp);
    if (ok) {
        ROS_INFO("spawned a turtle named %s", resp.name.c_str());
    } else {
        ROS_ERROR("Failed to spawn");
    }

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    tf::TransformListener listener;

    ros::Rate loop_rate(10);
    while (nh.ok()) {
        tf::StampedTransform tf_data;
        try {
			listener.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(3.0));
			listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), tf_data);            
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist vel;
        vel.linear.x = 0.5 * sqrt(pow(tf_data.getOrigin().x(), 2) + pow(tf_data.getOrigin().y(), 2));
        vel.angular.z = 4.0 * atan2(tf_data.getOrigin().y(), tf_data.getOrigin().x());

        // 下面的改动，可以让turtle1控制turtle2的转向，即turtle2跟随turtle1转，但turtle2追不上turtle1
        // double roll, pitch, yaw;
        // tf::Quaternion qua = tf_data.getRotation();
        // tf::Matrix3x3(qua).getRPY(roll, pitch, yaw);
        // vel.angular.z = 4.0 * yaw;

        cmd_pub.publish(vel);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}


