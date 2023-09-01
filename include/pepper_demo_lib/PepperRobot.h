#ifndef PEPPER_DEMO_LIB_PR2ROBOT_H
#define PEPPER_DEMO_LIB_PR2ROBOT_H

#include "pepper_demo_lib/Types.h"

#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <nao_interaction_msgs/MotionInterpolate.h>
#include <nao_interaction_msgs/String.h>
#include <nao_interaction_msgs/Say.h>
#include <nao_interaction_msgs/TrackerLookAt.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_listener.h"

class PepperRobot
{
protected:
  ros::ServiceClient interpolate_srv_;
  ros::ServiceClient look_at_srv_;

  ros::NodeHandle n_;
  ros::ServiceClient set_lang_srv_;
  ros::ServiceClient tts_srv_;

  tf2_ros::Buffer tf_buffer_; 
  tf2_ros::TransformListener tf_listener_;

  std::string result_;
  std::chrono::time_point<std::chrono::steady_clock> time_start_;
  std::chrono::time_point<std::chrono::steady_clock> time_stop_;

  std::condition_variable condition_variable_;

  bool is_real_;
  std::string lang_;
  std::map<std::string, std::string> en2fr_;

public:

  PepperRobot(bool real = true);
  ~PepperRobot();

  void initPose();

  void say(const std::string& txt);

  void openHand() { /* TODO */ }
  void closeHand() { /* TODO */ }

  void moveArm(const std::vector<std::vector<double>>& positions, bool right = true, double duration = 1.1);
  void moveArms(const std::vector<std::vector<double>>& left, const std::vector<std::vector<double>>& right, double duration = 1.1);

  void setLang(const std::string& lang);

  void startChrono() { time_start_ = std::chrono::steady_clock::now(); }
  void stopChrono() { time_stop_ = std::chrono::steady_clock::now(); }
  std::chrono::duration<double> getChrono() { return time_stop_ - time_start_; }

private:

  //void moveGripper(GripperState_e state);
  void lookAt(geometry_msgs::Vector3 point, double speed = 0.0);
};

#endif // PEPPER_DEMO_LIB_PR2ROBOT_H