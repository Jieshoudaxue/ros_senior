#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include "cxxopts.hpp"

void poseCb(const turtlesim::PoseConstPtr& msg, tf::TransformBroadcaster& br, const std::string& turtle_name) {
    tf::Transform tf_data;
    tf_data.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    tf_data.setRotation(q);

    br.sendTransform(tf::StampedTransform(tf_data, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "ycao_tf_broadcaster");

    // https://github.com/jarro2783/cxxopts
    cxxopts::Options options(argv[0], "specify turtle name");
    options.add_options()
        ("n,name", "Name of the turtle", cxxopts::value<std::string>())
        ("h,help", "show help");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        ROS_INFO("%s", options.help().c_str());
        return 0;
    }

    if (!result.count("name")) {
        ROS_ERROR("please specify turtle name using -n");
        return -1;
    }     

    std::string turtle_name = result["name"].as<std::string>();
    std::string pose_topic = turtle_name+"/pose";
    tf::TransformBroadcaster br;


    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>(pose_topic, 10, std::bind(poseCb, std::placeholders::_1, std::ref(br), turtle_name));

	ros::spin();

    return 0;
}