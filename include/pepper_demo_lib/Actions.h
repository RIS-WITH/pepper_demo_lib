#ifndef PEPPER_DEMO_LIB_ACTIONS_H
#define PEPPER_DEMO_LIB_ACTIONS_H

#include <string>
#include <vector>

#include "pepper_demo_lib/Types.h"

struct action_t
{
  action_t()
  {
    hand = hand_none;
    head = head_none;
    single_int = 0;
    single_double = 0.0;
  }

  std::string type;
  std::vector<std::vector<double>> rarm_position;
  std::vector<std::vector<double>> larm_position;
  bool follow_hand;
  bool invert_hand;
  HandState_e hand;
  HeadState_e head;
  std::string speak_str;
  int single_int;
  double single_double;
  double second_double;
  std::string ip_ws;
};

action_t createAction(const std::vector<std::vector<double>>& rarm_position, bool follow_hand = false, bool invert_hand = false, double speed = 0);

action_t createAction(const std::vector<std::vector<double>>& larm_position, const std::vector<std::vector<double>>& rarm_position, double duration = 1.1);

action_t createAction(HandState_e hand);

action_t createAction(HeadState_e head);

action_t createActionLookHand(bool invert);

action_t createAction(int speak_sound);

action_t createActionSpeak(const std::string& msg);
action_t createActionSpeakAnim(const std::string& msg);

action_t createActionDelay(int delay_ms);

action_t createActionResetBody();

action_t createActionMoveFront(double dist);
action_t createActionMoveRight(double dist);
action_t createActionMove(double dist_x, double dist_y);
action_t createActionTurn(double angle);

action_t createActionStartChrono();
action_t createActionStopChrono();

action_t createActionSynchro(const std::string& ip_addr);
action_t createActionLaunchSynchro(const std::string& ip_addr);
action_t createActionWaitSynchro(const std::string& ip_addr);

#endif // PEPPER_DEMO_LIB_ACTIONS_H