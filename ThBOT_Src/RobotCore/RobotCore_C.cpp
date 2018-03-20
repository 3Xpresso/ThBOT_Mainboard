// Odometry_C.cpp - extern C function definitions

#include "RobotCore_C.h"
#include "RobotCore.h"

extern void *RobotCore_C_new(void) {
    return new RobotCore();
}


