#include "pepper_demo_lib/PepperRobot.h"

PepperRobot::PepperRobot(bool real) : tf_listener_(tf_buffer_)
{
  is_real_ = real;

  interpolate_srv_ = n_.serviceClient<nao_interaction_msgs::MotionInterpolate>("naoqi_driver/motion/angle_interpolation");

  set_lang_srv_ = n_.serviceClient<nao_interaction_msgs::String>("naoqi_driver/tts/set_language");
  tts_srv_ = n_.serviceClient<nao_interaction_msgs::Say>("naoqi_driver/tts/say");
  look_at_srv_ = n_.serviceClient<nao_interaction_msgs::TrackerLookAt>("/naoqi_driver/tracker/look_at");

  lang_ = "fr";
  std::cout << "PepperRobot is ready" << std::endl;
}

PepperRobot::~PepperRobot()
{}

void PepperRobot::initPose()
{
  closeHand();
  moveArms({{ 1.242437720298767, 0.036568351089954376, -1.1184180974960327, -0.5266309380531311, -1.0351097583770752 }}, 
           {{ 1.3179131746292114, -0.03515475615859032, 1.3730974197387695, 0.6137155294418335, 0.8069871068000793 }}, 2);
}

void PepperRobot::say(const std::string& txt)
{
  nao_interaction_msgs::Say srv;
  if(lang_ == "en")
    srv.request.text = txt;
  else if(en2fr_.find(txt) != en2fr_.end())
    srv.request.text = en2fr_[txt];
  else
    srv.request.text = txt;
  tts_srv_.call(srv);
}

void PepperRobot::moveArm(const std::vector<std::vector<double>>& positions, bool right, double duration)
{
  nao_interaction_msgs::MotionInterpolate srv;
  srv.request.names.push_back(std::string(right ? "R" : "L") + "ShoulderPitch");
  srv.request.names.push_back(std::string(right ? "R" : "L") + "ShoulderRoll");
  srv.request.names.push_back(std::string(right ? "R" : "L") + "ElbowYaw");
  srv.request.names.push_back(std::string(right ? "R" : "L") + "ElbowRoll");
  srv.request.names.push_back(std::string(right ? "R" : "L") + "WristYaw");

  srv.request.angles.resize(srv.request.names.size());
  for(size_t i = 0; i < srv.request.names.size(); i++)
    for(auto& position : positions)
      srv.request.angles[i].values.push_back(position[i]);

  srv.request.times.resize(srv.request.names.size());
  for(size_t i = 0; i < srv.request.names.size(); i++)
  {
    float time = 0;
    for(auto& position : positions)
    {
      time += duration;
      srv.request.times[i].values.push_back(time);
    }
  }

  srv.request.is_absolute = true;

  interpolate_srv_.call(srv);
}

void PepperRobot::moveArms(const std::vector<std::vector<double>>& left, const std::vector<std::vector<double>>& right, double duration)
{
  nao_interaction_msgs::MotionInterpolate srv;
  srv.request.names.push_back("LShoulderPitch");
  srv.request.names.push_back("LShoulderRoll");
  srv.request.names.push_back("LElbowYaw");
  srv.request.names.push_back("LElbowRoll");
  srv.request.names.push_back("LWristYaw");
  srv.request.names.push_back("RShoulderPitch");
  srv.request.names.push_back("RShoulderRoll");
  srv.request.names.push_back("RElbowYaw");
  srv.request.names.push_back("RElbowRoll");
  srv.request.names.push_back("RWristYaw");

  size_t nb_joint = srv.request.names.size();

  std::cout << "names done" << std::endl;

  srv.request.angles.resize(nb_joint);
  for(size_t i = 0; i < nb_joint / 2; i++)
    for(auto& position : left)
      srv.request.angles[i].values.push_back(position[i]);

  std::cout << "angles left done" << std::endl;

  for(size_t i = 0; i < nb_joint / 2; i++)
    for(auto& position : right)
      srv.request.angles[nb_joint / 2 + i].values.push_back(position[i]);

  std::cout << "angles right done" << std::endl;

  srv.request.times.resize(nb_joint);
  for(size_t i = 0; i < nb_joint; i++)
  {
    float time = 0;
    for(auto& position : right)
    {
      time += duration;
      srv.request.times[i].values.push_back(time);
    }
  }

  std::cout << "times done" << std::endl;

  srv.request.is_absolute = true;

  interpolate_srv_.call(srv);
}

void PepperRobot::setLang(const std::string& lang)
{
  lang_ = lang;
  nao_interaction_msgs::String srv;
  srv.request.request = lang;
  set_lang_srv_.call(srv);
}

/*void PepperRobot::moveGripper(GripperState_e state)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal gripper_goal;
  gripper_goal.command.position = state == gripper_open ? 0.08 : 0.0;
  gripper_goal.command.max_effort = state == gripper_open ? -1.0 : 50.0;  
  
  ROS_INFO("Sending gripper goal");
  gripper_client_->sendGoal(gripper_goal);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper operated!");
  else
    ROS_INFO("The gripper failed.");
}*/

void PepperRobot::lookAt(geometry_msgs::Vector3 point, double speed)
{
  nao_interaction_msgs::TrackerLookAt srv;
  srv.request.target = point;
  srv.request.frame = 1; // world frame
  srv.request.max_speed_fraction = 0.5;
  srv.request.use_whole_body = false;

  look_at_srv_.call(srv);
}