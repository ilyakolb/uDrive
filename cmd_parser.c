#include "cmd_parser.h"

// 3-7-18: maybe todo: when sending no-arg command, args from previous command are still populated
//      not a big deal for now
void parseCmd(char cmdString[], struct CMD_STRUCT *s){
    int paramIter = 0;
    for (char *p = strtok(cmdString," "); p != NULL; p = strtok(NULL, " ")){
        
        if (paramIter == 0){
            if(stricmp(p, "rel") == 0) // syntax: rel 100 (rel move)
                s->c = CMD_REL;
            else if(stricmp(p, "abs") == 0) // syntax: abs 100 (abs move)
                s->c = CMD_ABS;
            else if(stricmp(p, "pos") == 0) // syntax: pos 4 (get position of actuator #4)
                s->c = CMD_P;
            else if(stricmp(p,"s") == 0) // syntax: s (stop)
                s->c = CMD_S;
            else if(stricmp(p,"setA") == 0) // syntax: setA 6 1 (set probe 6 to active [activeMask])
                s->c = CMD_SETA;
            else if(stricmp(p,"setM") == 0) // syntax: setM 6 1 (set probe 6 to one of the probes that move [moveMask])
                s->c = CMD_SETM;
            else if(stricmp(p,"z") == 0) // syntax: z (set position of active probe(s) to 0)
                s->c = CMD_ZERO;
            else if(stricmp(p,"drvPeak") == 0) // syntax: drvPeak 127 (set peak height to 127 [max])
                s->c = CMD_DRV_PEAK;
            else if(stricmp(p,"drvGain") == 0) // syntax: drvGain 3 (set piezo gain to maximum [100 VPP])
                s->c = CMD_DRV_GAIN;
            else if(stricmp(p,"piezoontime") == 0) // syntax: piezoontime 1000 (piezo on for 1,000 ms)
                s->c = CMD_DRV_PIEZOONTIME;
            else if(stricmp(p,"heaterpretime") == 0) // syntax: heaterpretime 5 (heater on for 5 ms before piezo)
                s->c = CMD_HEATER_PRETIME;
            else if(stricmp(p,"heaterposttime") == 0) // syntax: heaterposttime 5 (heater on for 5 ms after piezo)
                s->c = CMD_HEATER_POSTTIME;
            else if(stricmp(p,"ledgain") == 0) // syntax: ledgain 1 (set LED driver chip gain to max [1])
                s->c = CMD_HEATER_GAIN;
            else if(stricmp(p,"ledpwr") == 0) // syntax: ledpwr 63 (set LED power to 63 (max value))
                s->c = CMD_HEATER_PWR;
            else if(stricmp(p,"toggleheater") == 0) // syntax: toggleheater 5 1 (toggles 5th heater on/off on top board)
                s->c = CMD_HEATER_TOGGLE;
            else if(stricmp(p,"getA") == 0) // syntax: getA (returns 0 0 1 0 1 0 0 ... 24 bits corresponding to active channels)
                s->c = CMD_GET_ACTIVE;
            else if(stricmp(p,"getM") == 0) // syntax: getA (returns 0 0 1 0 1 0 0 ... 24 bits corresponding to movable channels)
                s->c = CMD_GET_MOVABLE;
            else if(stricmp(p,"mstatus") == 0) // syntax: mstatus
                s->c = CMD_MOTION_STATUS;
            else if(stricmp(p,"domove") == 0) // syntax: domove (takes one step with current active probes and params)
                s->c = CMD_DOMOVE;
            else if(stricmp(p,"comm") == 0) // syntax: comm (returns communication test result (see main))
                s->c = CMD_COMM_CHECK;
            else if(stricmp(p,"ping") == 0) // syntax: ping (returns ! character if successful)
                s->c = CMD_PING;
            
            // ADD COMMANDS HERE
            else // unrecognized command
                s->c = CMD_NONE;
        }
        else if(paramIter == 1){
            s->p1.paramI = atoi(p); // int
        }
        else if(paramIter == 2){
            //printf("parsed: %d\n", atoi(p));
            s->p2.paramI = atoi(p); // int
        }
        
        paramIter++;
    }
}

// TODO: move the errorOut to within functions
int execCmd (struct CMD_STRUCT *s){
    errorOut = 1; // by default, return error (1)
    
    printf("exec: %d %d %d \n", s->c, s->p1.paramI, s->p2.paramI);
    switch(s->c){
        case CMD_REL: errorOut = startRelMove(s->p1.paramI); break;
        case CMD_ABS: errorOut = startAbsMove(s->p1.paramI); break;
        case CMD_P: errorOut = getPosition(s->p1.paramI); break;
        case CMD_ZERO: errorOut = zeroPosition(); break;
        
        //drv
        case CMD_DRV_PEAK: errorOut = setDrvPeak(s->p1.paramI); break;
        case CMD_DRV_GAIN: errorOut = setDrvGain(s->p1.paramI); break;
        case CMD_DRV_PIEZOONTIME: errorOut = setDrvPiezoOnTime(s->p1.paramI); break;
        
        //heaters
        case CMD_HEATER_PRETIME: errorOut = setHeaterPreTime(s->p1.paramI); break;
        case CMD_HEATER_POSTTIME: errorOut = setHeaterPostTime(s->p1.paramI); break;
        case CMD_HEATER_GAIN: errorOut = setLEDBrightnessRange(s->p1.paramI, s->p2.paramI); break;
        case CMD_HEATER_PWR: errorOut = setLEDPwr(s->p1.paramI, s->p2.paramI); break;
        case CMD_HEATER_TOGGLE: errorOut = setHeaterToggle(s->p1.paramI, s->p2.paramI); break;
        //misc
        case CMD_SETA: errorOut = setActive(s->p1.paramI, s->p2.paramI);  break;
        case CMD_SETM: errorOut = setMoveMask(s->p1.paramI, s->p2.paramI); break;
        case CMD_GET_ACTIVE: errorOut = getActive(); break;
        case CMD_GET_MOVABLE: errorOut = getMoveMask(); break;
        case CMD_MOTION_STATUS: errorOut = getMotionStatus(); break;
        case CMD_DOMOVE: errorOut = doMove(); break;
        case CMD_COMM_CHECK: errorOut = commCheck(); break;
        case CMD_PING: printf("!\n"); break;
        default: printf("UNRECOGNIZED COMMAND!\n"); 
        
        // TO IMPLEMENT:
        // drv: gain, on time, peak val, DEMO: motor step
        // heaters: pre time, post time, heater gain, DEMO: specific heater on/off
        // misc: get active probes, get movable probes, motion status, communications check
        
    }
    
    return errorOut;
}