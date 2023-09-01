#ifndef PEPPER_DEMO_LIB_TASK_H
#define PEPPER_DEMO_LIB_TASK_H

#include "pepper_demo_lib/Actions.h"
#include "pepper_demo_lib/PepperRobot.h"

typedef std::vector<action_t> Task;

void executeAction(PepperRobot* robot, const action_t& action);

void executeTask(PepperRobot* robot, const std::vector<action_t>& task);

#endif // PEPPER_DEMO_LIB_TASK_H