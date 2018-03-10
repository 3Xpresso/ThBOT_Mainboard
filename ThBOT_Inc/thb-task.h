// Each task is assigned a priority from 0 to ( configMAX_PRIORITIES - 1 ),
// where configMAX_PRIORITIES is defined within FreeRTOSConfig.h.
// #define configMAX_PRIORITIES                     ( 7 )
#include "cmsis_os.h"

/// Priority used for thread control.
/// \note MUST REMAIN UNCHANGED: \b osPriority shall be consistent in every CMSIS-RTOS.
//typedef enum  {
//  osPriorityIdle          = -3,          ///< priority: idle (lowest)
//  osPriorityLow           = -2,          ///< priority: low
//  osPriorityBelowNormal   = -1,          ///< priority: below normal
//  osPriorityNormal        =  0,          ///< priority: normal (default)
//  osPriorityAboveNormal   = +1,          ///< priority: above normal
//  osPriorityHigh          = +2,          ///< priority: high
//  osPriorityRealtime      = +3,          ///< priority: realtime (highest)
//  osPriorityError         =  0x84        ///< system cannot determine priority or thread has illegal priority
//} osPriority;


#define	PRIORITY_IDLE		osPriorityIdle
#define	PRIORITY_UART		osPriorityNormal
#define	PRIORITY_MOTIONCTRL	osPriorityAboveNormal
#define	PRIORITY_ODOMETRY	osPriorityHigh

