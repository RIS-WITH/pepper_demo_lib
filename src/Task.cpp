#include "pepper_demo_lib/Task.h"

void executeAction(PepperRobot* robot, const action_t& action)
{
  std::cout << "in executeAction: " << action.type << std::endl;
  if(action.type == "arm")
  {
    robot->moveArm(action.rarm_position, true, action.single_double);
  }
  else if(action.type == "arms")
  {
    robot->moveArms(action.larm_position, action.rarm_position, action.single_double);
  }
  else if(action.type == "hand")
  {
    if(action.hand == hand_open)
      robot->openHand();
    else
      robot->closeHand();
  }
  /*else if(action.type == "head")
  {
    if(action.head == head_front)
      robot->lookFront();
  }*/
  else if(action.type == "speak")
    robot->say(action.speak_str);
  else if(action.type == "delay")
    usleep(action.single_int * 1000);
  else if(action.type == "startChrono")
    robot->startChrono();
  else if(action.type == "stopChrono")
    robot->stopChrono();
  /*else if(action.type == "lookHand")
  {
    if(action.invert_hand)
      robot->lookInvertHand(true);
    else
      robot->lookHand(true);
  }*/
}

void executeTask(PepperRobot* robot, const std::vector<action_t>& task)
{
  for(const auto& action : task)
    executeAction(robot, action);
}