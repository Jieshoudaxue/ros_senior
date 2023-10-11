#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <robot_voice/StringToVoice.h>


class RobotController {
public:
  RobotController() {
    ROS_INFO("RobotController Constructor");
  }

  ~RobotController() {
    ROS_INFO("RobotController Destructor");
  }

  int Init(ros::NodeHandle& nh) {
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    client_ = nh.serviceClient<robot_voice::StringToVoice>("str2voice");
    return 0;
  }

  void ToDownstream(const std::string& answer_txt, float linear_x, float angular_z) {
    robot_voice::StringToVoice::Request req;
    robot_voice::StringToVoice::Response resp;
    req.data = answer_txt;

    bool ok = client_.call(req, resp);
    if (ok) {
      printf("send str2voice service success: %s, and pub cmd_vel\n", req.data.c_str());
      for (int i = 0; i < 10; i ++) {
        geometry_msgs::Twist msg;
        msg.linear.x = linear_x;
        msg.angular.z = angular_z;
        cmd_pub_.publish(msg);
        sleep(0.5);
      }
    } else {
      ROS_ERROR("failed to send str2voice service");
    }
  }

  bool ChatterCallbback(robot_voice::StringToVoice::Request &req, robot_voice::StringToVoice::Response &resp) {
    printf("i received: %s\n", req.data.c_str());

    std::string voice_txt = req.data;

    if (voice_txt.find("前") != std::string::npos) {
      ToDownstream("小车请向前跑", 0.3, 0);
    } else if (voice_txt.find("后") != std::string::npos) {
      ToDownstream("小车请向后倒", -0.3, 0);
    } else if (voice_txt.find("左") != std::string::npos) {
      ToDownstream("小车请向左转", 0, 0.3);
    } else if (voice_txt.find("右") != std::string::npos) {
      ToDownstream("小车请向右转", 0, -0.3);
    } else if (voice_txt.find("转") != std::string::npos) {
      ToDownstream("小车请打转", 0.3, -0.3);
    }

    resp.success = true;

    return resp.success;
  }

  void Start(ros::NodeHandle& nh) {
    chatter_server_ = nh.advertiseService("human_chatter", &RobotController::ChatterCallbback, this);
  }

private:
  ros::ServiceServer chatter_server_;
  ros::Publisher cmd_pub_;
  ros::ServiceClient client_;
};


int main(int argc, char* argv[]) {
  int ret = 0;
  ros::init(argc, argv, "voice_controller");
  ros::NodeHandle nh;

  RobotController rc;
  rc.Init(nh);

  printf("this is a voice controller app for robot, you can say: 向前, 向后, 向左, 向右, 转圈, 结束\n");
  rc.Start(nh);

  ros::spin();

  return 0;
}
