/* Odometry_C.h - must compile in both C and C++ */

#ifndef Odometry_C_H
#define Odometry_C_H

#ifdef __cplusplus
    extern "C" {
#endif

extern void *Odometry_C_new(void);
extern void Odometry_C_init(void *odometry);

#ifdef __cplusplus
}
#endif

#endif

