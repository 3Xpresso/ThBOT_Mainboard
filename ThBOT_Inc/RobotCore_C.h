/* RobotCore_C.h - must compile in both C and C++ */

#ifndef RobotCore_C_H
#define RobotCore_C_H

#ifdef __cplusplus
    extern "C" {
#endif

typedef struct RobotCore RobotCore;

extern RobotCore *RobotCore_C_new(void);
extern void RobotCore_C_init(RobotCore *robocore);

extern void RobotCore_C_task(RobotCore *robocore);

#ifdef __cplusplus
}
#endif

#endif

