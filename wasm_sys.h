#pragma once

#ifndef WASMSYS
#define WASMSYS
/*currently not in use will be used at a later stage for file structure improvements and mainly with in-process wasmmodule integration
#ifndef __INTELLISENSE__
#	define MODULE_EXPORT __attribute__( ( visibility( "default" ) ) )
#	define MODULE_WASM_MODNAME(mod) __attribute__((import_module(mod)))
#else
#	define MODULE_EXPORT
#	define MODULE_WASM_MODNAME(mod)
#	define __attribute__(x)
#	define __restrict__
#endif*/
#include "common_sys.h"

#include "systems/elec_sys.h"
#include "systems/packs_sys.h"
#include "systems/bleed_sys.h"
#include "systems/press_sys.h"
#include "systems/eng_sys.h"
#include "systems/hyd_sys.h"


class WasmSys {
private:
    ElecSys ELEC_SYSTEM;
    PacksSys PACK_SYSTEM;
    BleedSys BLEED_SYSTEM;
    PressSys PRESS_SYSTEM;
    EngSys ENG_SYSTEM;
    HydSys HYD_SYSTEM;
public:
    void init() {
        srand(time(nullptr));
        initUnitEnums();
        initLocalSimVarsIDs();
        initLSimVarsUnit();
        ELEC_SYSTEM.init();
        PACK_SYSTEM.init();
        BLEED_SYSTEM.init();
        PRESS_SYSTEM.init();
        ENG_SYSTEM.init();
        HYD_SYSTEM.init();
        updateSetLSimVars();
    }
    void update(double const currentAbsTime) {
        updateASimVars();
        updateGetLSimVars();

        lSimVarsValue[FLIGHT_PHASE] = flightPhase();

        ELEC_SYSTEM.update(currentAbsTime);
        PACK_SYSTEM.update();
        BLEED_SYSTEM.update(currentAbsTime);
        PRESS_SYSTEM.update(currentAbsTime);
        ENG_SYSTEM.update(currentAbsTime);
        HYD_SYSTEM.update(currentAbsTime);
        updateSetLSimVars();

        lastAbsTime = currentAbsTime;
    }
    void destroy() {
        unregister_all_named_vars();
        free(ENUM_UNITS);
        free(ID_LSIMVAR);
    }
};
#endif