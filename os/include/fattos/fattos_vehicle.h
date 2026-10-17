#ifndef FATTOS_VEHICLE_H
#define FATTOS_VEHICLE_H

#include <stdbool.h>
#include <stdint.h>
#include "fattos.h"
#include "fattos_safety.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Vehicle state manager — the arming gate every autonomous vehicle needs,
 * sitting in front of the actuator safety supervisor:
 *
 *   INIT -> DISARMED -> (RC ok + arm switch held + prearm ok + supervisor
 *          healthy for arm_hold_ms) -> ARMED
 *   ARMED -> FAILSAFE   on RC loss, RC failsafe flag, or supervisor fault
 *   ARMED/FAILSAFE -> ESTOP on latched e-stop
 *   FAILSAFE -> DISARMED when healthy again (pilot must re-arm)
 *   ESTOP -> DISARMED only via explicit fattos_vehicle_estop_clear()
 *
 * Nothing actuates unless state == ARMED and the safety supervisor is
 * healthy (fattos_vehicle_can_actuate). This is the flight-controller
 * model: arming is deliberate, losing the pilot stops the vehicle, and
 * an e-stop is a latch, not a transient.
 */
typedef enum {
    FATTOS_VEHICLE_INIT = 0,
    FATTOS_VEHICLE_DISARMED,
    FATTOS_VEHICLE_ARMED,
    FATTOS_VEHICLE_FAILSAFE,
    FATTOS_VEHICLE_ESTOP
} FattosVehicleState;

typedef struct {
    FattosVehicleState state;
    FattosSafety *safety;     /* supervisor in front of the actuators, may be NULL */
    bool rc_present;          /* set by fattos_vehicle_set_rc while frames arrive */
    bool rc_arm_switch;       /* pilot arm request */
    bool prearm_ok;           /* app-level health (IMU converged, sensors sane) */
    bool arm_pending;         /* an arm hold is in progress */
    uint32_t arm_request_ms;  /* when the arm hold started */
    uint32_t arm_hold_ms;     /* arm switch must be held this long to arm */
    uint32_t last_rc_ms;      /* last valid RC update */
    uint32_t rc_timeout_ms;   /* RC loss -> FAILSAFE after this long */
    uint32_t state_changes;   /* transitions seen (diagnostics/telemetry) */
} FattosVehicle;

FattosStatus fattos_vehicle_init(FattosVehicle *vehicle, FattosSafety *safety,
                                 uint32_t arm_hold_ms, uint32_t rc_timeout_ms);

/* Feed RC state each control cycle (from the SBUS decoder or a GCS link). */
void fattos_vehicle_set_rc(FattosVehicle *vehicle, bool present,
                           bool arm_switch);
/* App-level pre-arm health: false blocks arming and drops to DISARMED. */
void fattos_vehicle_set_prearm(FattosVehicle *vehicle, bool ok);

/* Advance the state machine (one call per control cycle). Returns state. */
FattosVehicleState fattos_vehicle_update(FattosVehicle *vehicle);

/* True only when ARMED and the safety supervisor is healthy. Gate every
   actuator command on this — it is the single "may I move?" question. */
bool fattos_vehicle_can_actuate(const FattosVehicle *vehicle);

/* Latch an emergency stop (forwards to the safety supervisor). */
FattosStatus fattos_vehicle_estop(FattosVehicle *vehicle);
/* Explicit recovery: clears supervisor latch if healthy, -> DISARMED. */
FattosStatus fattos_vehicle_estop_clear(FattosVehicle *vehicle);

const char *fattos_vehicle_state_name(FattosVehicleState state);

#ifdef __cplusplus
}
#endif

#endif