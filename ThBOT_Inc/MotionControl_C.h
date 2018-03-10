/* MotionControl_C.h - must compile in both C and C++ */

#ifndef MotionControl_C_H
#define MotionControl_C_H

#ifdef __cplusplus
    extern "C" {
#endif

extern void *MotionControl_C_new(void);
extern void MotionControl_C_init(void *motioncontrol);

#ifdef __cplusplus
}
#endif

#endif

