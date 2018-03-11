#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "thb-fsm.h"
#include "thb-tests.h"

typedef struct Cmd
{
	uint32_t Mode;
	uint32_t State;
} t_cmd;

#define MAX_CMD          16

typedef struct CmdRec
{
	uint32_t RdIndex;
	uint32_t WrIndex;
	t_cmd st_Cmd[MAX_CMD];
} t_CmdRec;

static t_CmdRec st_CmdRec = {
	.RdIndex = 0,
	.WrIndex = 0,
	.st_Cmd = {
	},
};

typedef void (*fn_ptr) (uint32_t Evt);

typedef struct Mode
{
	fn_ptr fct[MAX_MODE];
} t_Mode;

static uint32_t Mode = MODE_IDLE;
static uint32_t CounterLoop = 0;

static void idle_callback(uint32_t Evt);

const t_Mode st_ModeCallBack = {
	.fct = {
		idle_callback,
		NULL,
		NULL,
		NULL,
		exec_test,
	},
};

static void idle_callback(uint32_t Evt)
{
    CounterLoop++;
    if (CounterLoop > 3000)
    {
        printf("Mode %i - State Idle\n", (int)Mode);
        CounterLoop = 0;
    }
}

static uint32_t get_state(void)
{
	uint32_t State = STATE_IDLE;

   	if (st_CmdRec.RdIndex != st_CmdRec.WrIndex)
    {
    	st_CmdRec.RdIndex++;
    	if (st_CmdRec.RdIndex >= MAX_CMD)
    		st_CmdRec.RdIndex = 0;
    }

   	Mode  = st_CmdRec.st_Cmd[st_CmdRec.RdIndex].Mode;
   	State = st_CmdRec.st_Cmd[st_CmdRec.RdIndex].State;

   	return State;
}

void thb_fsm_ChangeModeState(uint32_t Mode, uint32_t State)
{
	st_CmdRec.WrIndex++;
	if (st_CmdRec.WrIndex >= MAX_CMD)
		st_CmdRec.WrIndex = 0;

	st_CmdRec.st_Cmd[st_CmdRec.WrIndex].Mode  = Mode;
	st_CmdRec.st_Cmd[st_CmdRec.WrIndex].State = State;
}

void thb_fsm_loop(void)
{
	fn_ptr CallBack = NULL;
	uint32_t State;

	thb_fsm_ChangeModeState(MODE_IDLE, STATE_IDLE);

    for(;;)
    {
    	osDelay(1);

    	State = get_state();
    	CallBack = st_ModeCallBack.fct[Mode];
    	if (CallBack != NULL)
    		CallBack(State);
    	else
    		idle_callback(State);
    }
}