/*
 * thb-param.h
 *
 *  Created on: 16 mars 2018
 *      Author: jpb
 */

#ifndef THB_PARAM_H_
#define THB_PARAM_H_

enum {
  PARAM_NONE,
  PARAM_PWM,

  /***********/
  PARAM_MAX
};

void thb_param_SetParameter(uint32_t ParamId, char* ParamValue, uint32_t ValueLen);

void thb_param_SetPercentPower(uint32_t Percentage);
uint32_t thb_param_GetPercentPower(void);


#endif /* THB_PARAM_H_ */
