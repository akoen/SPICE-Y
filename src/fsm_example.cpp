// variables

typedef enum
{
    fsm_State1 = 0u,
    fsm_State2,
    fsm_State3,
    fsm_StateMaxNum
} fsm_states;

typedef enum
{
    fsm_flagTreasure = 0u,
    fsm_flagTape,
    fsm_flagIR
} fsm_flags;

void (*fsm_states[fsm_StateMaxNum])(void); // syntax: returns (*pointer to array[elements]) (args)
volatile int fsm_stateVal;
volatile int fsm_flag;

// fsm handler prototypes

void fsm_state1_handler(void)
{
    if (fsm_checkFlags(fsm_flagTreasure))
    {
        fsm_stateVal = fsm_State2; // once fn returns, fsm_update() will be called with fsm_State2 as index, so will go into state2 handler
    }
    else
    {
        // follow tape
    }
}

void fsm_state2_handler(void);
void fsm_state3_handler(void);

// function implementations

void fsm_init(void)
{
    // indicate starting state
    fsm_stateVal = fsm_State1;
    // populate function pointer array with handler addresses
    fsm_states[fsm_State2] = &fsm_state2_handler;
    fsm_states[fsm_State3] = &fsm_state3_handler;
    fsm_states[fsm_State1] = &fsm_state1_handler;
}

void fsm_update(void)
{
    // call state handler
    (void)(*fsm_states[fsm_stateVal]);

    // "manually" transition to next state if needed (other options to transition between states)
    // fsm_stateVal++;

    // what to do when state reaches max states
    if (fsm_stateVal == fsm_StateMaxNum)
    {
        fsm_stateVal = fsm_State1;
    }
}

int fsm_checkFlags(int fsm_flag)
{
    /*if (fsm_flag == something){
        fsm_stateVal = next_state;
    }
    */
}

/* to give an idea, we'd ONLY have this code in our main loop
void loop()
{
    fsm_update();
}
*/