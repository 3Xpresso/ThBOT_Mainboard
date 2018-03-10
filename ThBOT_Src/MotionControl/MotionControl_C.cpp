// MotionControl_C.cpp - extern C function definitions

#include "MotionControl_C.h"
#include "MotionControl.h"

extern void *MotionControl_C_new(void) {
    return new MotionControl();
}
