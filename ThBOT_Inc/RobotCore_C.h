/* RobotCore_C.h - must compile in both C and C++ */

#ifndef RobotCore_C_H
#define RobotCore_C_H

#ifdef __cplusplus
    extern "C" {
#endif

extern void *RobotCore_C_new(void);
extern void RobotCore_C_init(void *robocore);

#ifdef __cplusplus
}
#endif

#endif

