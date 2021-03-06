# A32NX_WASM_SYSTEM
Core systems implementation for flybywire_A32NX as wasm module.

## Documentation
- Simconnect documentation and gauge documentation available with the MSFS SDK is self sufficient.

## Steps to add new system
- Create all your system implementation(as classes) in its own header file with a master class so that you don't flood `wasm_sys.cpp`
  - Expose only `init()`, `update()`, (NOTE: `updateSimVars()` has been removed)
  - Create a class instance under `wasm_sys.h`, WasmSys master class(qualified as private and not public) and call your system's `init()`, `update()` function within the wasmSys class's public `init()` and `update()` function declarations.
    - currently implemented as wasm_guage, moving forward it'll be ported to wasm in-process module. You don't have to worry about porting anything/changing anything, it'll be a single function call update.
    - Current implementation uses wasm module run absTime to calulate _deltaTime instead of directly using deltaTime from pData structure, this is done keeping the above
      point under consideration, since _deltaTime is bound to gauge refresh rate.(Note: currently the difference between currentAbsTime and lastAbsTime will be equal to            REFRESH_RATE defined under common_sys.h)
    
- `#include "common_sys.h"` in your header file. 
  - Use and update `enum` definition and `PCSTRINGZ` definitions under `data/data_enums.h` and `data/data_strings.h` as required(do not delete/change order of any existing ones)
  - In order to add new ones, first add its enum to the enum definitions and follow suit with its string at the same index as enum definition
  - Ensure to provide a comment of local var name to be used while defining its enum(yes, one may scroll down to PCSTRINGZ array and lookup,
    but this makes things easier for others to work with.)
    
  - Incase you need systems to operate at a refresh rate faster than the current `REFRESH_RATE` defined under `common_sys.h`, create a new refresh timer exec block under wasm callbacks and use it. Unless the system _REALLY_ needs to be run at a higher refresh rate, refrain from doing so.
- Thread support is currently unavailable so expect sync delay issues with update function(worst case should be no more than one update cycle desync).
- Once thread support is available all the functions can then run async.
    
## SDK Third party interaction
- Third party interaction can be achieved through transmitting events for timebeing, client data space interaction maybe added in the future.
  - EventIDs for L: Vars needs to be set as follows `THIRD_PARTY_EVENT_ID_MIN /*(hex value for this is 0x00011000)*/ + <lSimVars_enum>`, lSimVars enum can be found under `data/data_enums.h` 
  - `SimConnect_MapClientEventToSimEvent` to register event.
  - `SimConnect_TransmitClientEvent` to transmit event with value of type `DWORD`(double) specified for dwData parameter.
  - refer to MSFS simconnect SDK for more details on these functions.

