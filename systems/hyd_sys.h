#pragma once
#include "../common_sys.h"

class GreenLoop {
private:
    bool priorityValve;
    float EDP1Displacement, EDP1Flow, accPressure, accVolume;
    const float accuPreCharge = 1885, accMaxVolume = 0.241966, maxPipeV = 1.09985;

    float calculateEDP1Flow() {
        if (lSimVarsValue[GREEN_PRESSURE] < 2900) EDP1Displacement = 2.4;
        else EDP1Displacement = max((-0.0192 * lSimVarsValue[GREEN_PRESSURE] + 58.08), 0);
        EDP1Flow = ((((aSimVarsValue[ENG1_MAX_RPM_PCT] * 4000) * EDP1Displacement) / 231) / 60) * (deltaT * 0.0001);
        float x = lSimVarsValue[GREEN_RES_VOLUME] - EDP1Flow;
        if (x > 0) {
            lSimVarsValue[GREEN_RES_VOLUME] = x;
        }
        else {
            EDP1Flow = lSimVarsValue[GREEN_RES_VOLUME];
            lSimVarsValue[GREEN_RES_VOLUME] = 0;
        };
        return EDP1Flow;
    };

    void calculateGreenPressure() {
        float deltaV = lSimVarsValue[GREEN_FLOW] - lSimVarsValue[GREEN_LOAD];       //WIP
        float deltaP = 0;
        if (deltaV > 0) {
            if (lSimVarsValue[GREEN_VOLUME] < maxPipeV) {
                float x = maxPipeV - (lSimVarsValue[GREEN_VOLUME] + deltaV);
                if (x > 0) {
                    lSimVarsValue[GREEN_VOLUME] += deltaV;
                    deltaV = 0;
                }
                else {
                    lSimVarsValue[GREEN_VOLUME] = maxPipeV;
                    deltaV = abs(x);
                };
            };     
            if (accPressure < 3000 && deltaV > 0) {
                float x = 0.08993 - (accVolume + deltaV); //x = volume required to reach accu ~3000psi - Pre-calculated
                if (x > 0) {
                    accVolume += deltaV;
                    accPressure = (accuPreCharge * accMaxVolume) / (accMaxVolume - accVolume);
                }
                else {  //If deltaV is more than the acc can take to get to 3,000 , whatever is excess is compressed and creates pressure
                    accVolume = 0.08993;
                    accPressure = 3000;
                    deltaP = ((abs(x) * 250000) / lSimVarsValue[GREEN_VOLUME]); //WIP - Rate too high or refresh rate of system too slow; pressure increases too quickly per tick for displacement to knock down flowrate
                    lSimVarsValue[GREEN_VOLUME] += abs(x);
                };
            }
            else {
                deltaP = ((deltaV * 250000) / lSimVarsValue[GREEN_VOLUME]); //WIP - Same as mentioned above
                lSimVarsValue[GREEN_VOLUME] += deltaV;
            };
        };
        if (deltaV < 0) {
            if (accVolume > 0) {
                float x = deltaV + accVolume;
                if (x > 0) {
                    deltaV = 0;
                    deltaP -= 2;                //Placeholder load
                    accVolume += deltaV;
                    accPressure = (accuPreCharge * accMaxVolume) / (accMaxVolume - accVolume);
                }
                else {
                    deltaV = x;
                    accVolume = 0;
                    accPressure = accuPreCharge;
                };
            }

            float x = (lSimVarsValue[GREEN_VOLUME] + deltaV) - maxPipeV;
            if (x > 0) {
                deltaP = ((deltaV * 250000) / lSimVarsValue[GREEN_VOLUME]);
            }
            else {
                lSimVarsValue[GREEN_PRESSURE] = 0;
            };

            lSimVarsValue[GREEN_VOLUME] = max((lSimVarsValue[GREEN_VOLUME] + deltaV), 0);
        };
        if (deltaP != 0) lSimVarsValue[GREEN_PRESSURE] = max((lSimVarsValue[GREEN_PRESSURE] + deltaP), 0);
    };

public:
    void init() {
        EDP1Displacement = 2.4;
        lSimVarsValue[GREEN_VOLUME] = 1;
        lSimVarsValue[GREEN_RES_VOLUME] = 3.7;
        lSimVarsValue[GREEN_PRESSURE] = 0;
        lSimVarsValue[GREEN_FLOW] = 0;
        lSimVarsValue[GREEN_LOAD] = 0;
        priorityValve = false;
        accPressure = accuPreCharge;
        accVolume = 0;
    }
    void update(const double currentAbsTime) {
        lSimVarsValue[GREEN_FLOW] = 0;

        if (lSimVarsValue[EDP1_SWITCH]) lSimVarsValue[GREEN_FLOW] += calculateEDP1Flow();
        //Might add PTU flow/load check here?
        calculateGreenPressure();

    };
};

class YellowLoop {
private:
    const int ePumpTimeToRPM = 2;      //2 seconds for the electric pump to reach max RPM --Placeholder, requires engineer input
    bool priorityValve;
    float EDP2Displacement, EDP2Flow, ePumpDisplacement, ePumpRPM, ePumpFlow, accPressure, accVolume;
    const float accuPreCharge = 1885, accMaxVolume = 0.241966;

    float calculateEDP2Flow() {
        if (lSimVarsValue[YELLOW_PRESSURE] < 2900) EDP2Displacement = 2.4;
        else EDP2Displacement = max((-0.0192 * lSimVarsValue[YELLOW_PRESSURE] + 58.08), 0);
        EDP2Flow = ((((aSimVarsValue[ENG2_MAX_RPM_PCT] * 4000) * EDP2Displacement) / 231) / 60) * (deltaT * 0.0001);
        float x = lSimVarsValue[YELLOW_RES_VOLUME] - EDP2Flow;
        if (x >= 0) {
            lSimVarsValue[YELLOW_RES_VOLUME] = x;
        }
        else {
            EDP2Flow = lSimVarsValue[YELLOW_RES_VOLUME];
            lSimVarsValue[YELLOW_RES_VOLUME] = 0;
        };
        return EDP2Flow;
    };

    void startEPump(bool start) {
        if (start) ePumpRPM += min((7600 / ePumpTimeToRPM) * (deltaT * 0.0001), 7600);
        else ePumpRPM -= max((7600 / ePumpTimeToRPM) * (deltaT * 0.0001), 7600);
        lSimVarsValue[YELLOW_PUMP_ACTIVE] = start;
    };

    float calculateEPumpFlow() {           
        if (lSimVarsValue[YELLOW_PRESSURE] < 2900) ePumpDisplacement = 0.263;
        else ePumpDisplacement = max((-0.002104 * lSimVarsValue[YELLOW_PRESSURE] + 6.3646), 0);
        ePumpFlow = (((ePumpRPM * ePumpDisplacement) / 231) / 60) * (deltaT * 0.0001);
        float x = lSimVarsValue[YELLOW_RES_VOLUME] - ePumpFlow;
        if (x >= 0) {
            lSimVarsValue[YELLOW_RES_VOLUME] = x;
        }
        else {
            ePumpFlow = lSimVarsValue[YELLOW_RES_VOLUME];
            lSimVarsValue[YELLOW_RES_VOLUME] = 0;
        };
        return ePumpFlow;
    };
    void calculateYellowPressure() {
        float deltaV = lSimVarsValue[YELLOW_FLOW] - lSimVarsValue[YELLOW_LOAD];
        float deltaP = 0;
        if (deltaV > 0) {
            if (accPressure < 3000) {
                float x = 0.08993 - (accVolume + deltaV); 
                if (x > 0) {
                    accVolume += deltaV;
                    accPressure = (accuPreCharge * accMaxVolume) / (accMaxVolume - accVolume);
                }
                else {
                    accVolume = 0.08993;
                    accPressure = 3000;
                    deltaP = ((abs(x) * 250000) / lSimVarsValue[YELLOW_VOLUME]);
                    lSimVarsValue[YELLOW_VOLUME] += abs(x);
                }
            }
            else {
                deltaP = ((deltaV * 250000) / lSimVarsValue[YELLOW_VOLUME]);
                lSimVarsValue[YELLOW_VOLUME] += deltaV;
            }
        }
        if (deltaP > 0) lSimVarsValue[YELLOW_PRESSURE] = max(min(lSimVarsValue[YELLOW_PRESSURE] + deltaP, 3025), 0);
    };

public:
    void init() {
        EDP2Displacement = 2.4;
        ePumpDisplacement = 0.263;
        lSimVarsValue[YELLOW_VOLUME] = 3.2;
        lSimVarsValue[YELLOW_PRESSURE] = 0;
        lSimVarsValue[YELLOW_FLOW] = 0;
        lSimVarsValue[YELLOW_LOAD] = 0;
        priorityValve = false;
        accPressure = accuPreCharge;
        accVolume = 0;
    }
    void update() {
        lSimVarsValue[YELLOW_FLOW] = 0;

        if (lSimVarsValue[YELLOW_PUMP_SWITCH] || (aSimVarsValue[FWD_CARGO] > 0 && EDP2Flow == 0)) { //WIP
            if (aSimVarsValue[FWD_CARGO] > 0) priorityValve = true;
            if (ePumpRPM > 0) lSimVarsValue[YELLOW_FLOW] += calculateEPumpFlow();
            if (ePumpRPM < 7600 && (lSimVarsValue[AC_BUS2] != 0 || lSimVarsValue[EXT_GEN_ONLINE])) startEPump(true);
            else startEPump(false);
        }
        else if (ePumpRPM > 0) {
            startEPump(false);
        };

        if (lSimVarsValue[EDP2_SWITCH]) lSimVarsValue[YELLOW_FLOW] += calculateEDP2Flow();
        calculateYellowPressure();

    };
};

class BlueLoop {
private:
    const int ePumpTimeToRPM = 2;
    bool priorityValve;
    float ePumpDisplacement, ePumpRPM, ePumpFlow, accPressure, accVolume;
    const float accuPreCharge = 1885, accMaxVolume = 0.241966;

    void startEPump(bool start) {
        if (start) ePumpRPM += min((7600 / ePumpTimeToRPM) * (deltaT * 0.0001), 7600);
        else ePumpRPM -= max((7600 / ePumpTimeToRPM) * (deltaT * 0.0001), 7600);
        lSimVarsValue[BLUE_PUMP_ACTIVE] = start;
    }
    float calculateEPumpFlow() {
        if (lSimVarsValue[BLUE_PRESSURE] < 2900) ePumpDisplacement = 0.263;
        else ePumpDisplacement = max((-0.002104 * lSimVarsValue[BLUE_PRESSURE] + 6.3646), 0);
        ePumpFlow = (((ePumpRPM * ePumpDisplacement) / 231) / 60) * (deltaT * 0.0001);
        float x = lSimVarsValue[BLUE_RES_VOLUME] - ePumpFlow;
        if (x >= 0) {
            lSimVarsValue[BLUE_RES_VOLUME] = x;
        }
        else {
            ePumpFlow = lSimVarsValue[BLUE_RES_VOLUME];
            lSimVarsValue[BLUE_RES_VOLUME] = 0;
        };
        return ePumpFlow;
    };
    void calculateBluePressure() {
        float deltaV = lSimVarsValue[BLUE_FLOW] - lSimVarsValue[BLUE_LOAD];
        float deltaP = 0;
        if (deltaV > 0) {
            if (accPressure < 3000) {
                float x = 0.08993 - (accVolume + deltaV);
                if (x > 0) {
                    accVolume += deltaV;
                    accPressure = (accuPreCharge * accMaxVolume) / (accMaxVolume - accVolume);
                }
                else {
                    accVolume = 0.08993;
                    accPressure = 3000;
                    deltaP = ((abs(x) * 250000) / lSimVarsValue[BLUE_VOLUME]);
                    lSimVarsValue[BLUE_VOLUME] += abs(x);
                }
            }
            else {
                deltaP = ((deltaV * 250000) / lSimVarsValue[BLUE_VOLUME]);
                lSimVarsValue[BLUE_VOLUME] += deltaV;
            }
        }
        if (deltaP > 0) lSimVarsValue[BLUE_PRESSURE] = max(min(lSimVarsValue[BLUE_PRESSURE] + deltaP, 3025), 0);
    };


public:
    void init() {
        ePumpDisplacement = 0.263;
        lSimVarsValue[BLUE_VOLUME] = 2.6;
        lSimVarsValue[BLUE_PRESSURE] = 0;
        lSimVarsValue[BLUE_FLOW] = 0;
        lSimVarsValue[BLUE_LOAD] = 0;
        priorityValve = false;
        accPressure = accuPreCharge;
        accVolume = 0;
    }
    void update() {
        lSimVarsValue[BLUE_FLOW] = 0;

        if (lSimVarsValue[BLUE_PUMP_SWITCH]) {
            if ((aSimVarsValue[ENG1_MAX_RPM_PCT] >= 0.4 || aSimVarsValue[ENG2_MAX_RPM_PCT] >= 0.4 || !aSimVarsValue[ON_GROUND]) && lSimVarsValue[AC_BUS1] != 0) {
                if (ePumpRPM < 7600) startEPump(true);
            }
            else if (ePumpRPM > 0) startEPump(false);
            if (ePumpRPM > 0) lSimVarsValue[BLUE_FLOW] += calculateEPumpFlow();
        }
        else if (ePumpRPM > 0) startEPump(false);

        calculateBluePressure();
    };
};

class ErrorChecks {
private:

public:
    void init() {

    };
    void update(const double currentAbsTime) {

    };
    void updateSimVars() {

    };
};


class HydSys {
private:
    GreenLoop green;
    YellowLoop yellow;
    BlueLoop blue;

//    PTUFunctions ptu;
//    ErrorChecks errors;
public:
    void init() {
        green.init();
        yellow.init();
        blue.init();
//        errors.init();
//        trigger_key_event(KEY_HYDRAULIC_SWITCH_TOGGLE, 1);  //Work-around to force disable Asobo's default hydraulics behaviour, as removing pumps in the configs has no effect it seems :c   --TBC
//        trigger_key_event(KEY_HYDRAULIC_SWITCH_TOGGLE, 2);
    }
    void update(const double currentAbsTime) {
        green.update(currentAbsTime);
        yellow.update();
        blue.update();
 //       errors.update(currentAbsTime);
    }
};


/**
P valves 1841psi


Normal fill 14l
Max gageable 18L

Loop? 5L
**/

