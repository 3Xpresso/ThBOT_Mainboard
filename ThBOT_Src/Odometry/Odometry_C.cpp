// Odometry_C.cpp - extern C function definitions

#include "Odometry_C.h"
#include "Odometry.h"

extern void *Odometry_C_new(void) {
    return new Odometry();
}
