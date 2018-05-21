
#ifndef THB_FSM_H
#define THB_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
  STATE_IDLE,
  STATE_GET_PWR_STATE,
  STATE_ENC_READ,
  STATE_MOVE_FORWARD,
  STATE_MOVE_BACKWARD,
  STATE_TURN_RIGTH,
  STATE_TURN_LEFT,
  STATE_TURN_LEFT_45,
  STATE_TURN_LEFT_90,
  STATE_TURN_LEFT_180,
  STATE_TURN_LEFT_270,
  STATE_ODOM_PARAMS,
};

enum {
  MODE_IDLE,
  MODE_COMPET,
  MODE_QUALIF,
  MODE_CALIBR,
  MODE_TESTS,
  MAX_MODE
};

extern void thb_fsm_ChangeModeState(uint32_t Mode, uint32_t State);
extern void thb_fsm_loop(void);
extern void thb_fsm_StoreCmd(uint32_t Mode, uint32_t State);
extern void thb_fsm_GetModeState(uint32_t *Mode, uint32_t *State);

#ifdef __cplusplus
}
#endif

#endif /* THB_FSM_H */
