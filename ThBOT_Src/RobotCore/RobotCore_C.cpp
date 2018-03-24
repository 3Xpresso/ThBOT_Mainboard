// Odometry_C.cpp - extern C function definitions

#include "RobotCore.h"
#include "RobotCore_C.h"

extern RobotCore *RobotCore_C_new(void) {
    return new RobotCore();
}

extern void RobotCore_C_init(RobotCore *robocore) {
    return robocore->Init();
}

extern void RobotCore_C_task(RobotCore *robocore) {
    return robocore->Task();
}
