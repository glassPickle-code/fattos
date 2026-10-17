#include "fattos/fattos_vehicle.h"
#include "fattos/fattos_safety.h"
#include "fattos/fattos_sensor.h"

const char *fattos_vehicle_state_name(FattosVehicleState state)
{
    switch (state) {
    case FATTOS_VEHICLE_INIT:
        return "INIT";
    case FATTOS_VEHICLE_DISARMED:
        return "DISARMED";
    case FATTOS_VEHICLE_ARMED:
        return "ARMED";
    case FATTOS_VEHICLE_FAILSAFE:
        return "FAILSAFE";
    case FATTOS_VEHICLE_ESTOP:
        return "ESTOP";
    default:
        return "UNKNOWN";
    }
}

FattosStatus fattos_vehicle_init(FattosVehicle *vehicle, FattosSafety *safety,
                                 uint32_t arm_hold_ms, uint32_t rc_timeout_ms)
{
    if (vehicle == NULL || arm_hold_ms == 0U || rc_timeout_ms == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }
    vehicle->state = FATTOS_VEHICLE_INIT;
    vehicle->safety = safety;
    vehicle->rc_present = false;
    vehicle->rc_arm_switch = false;
    vehicle->prearm_ok = false;
    vehicle->arm_pending = false;
    vehicle->arm_request_ms = 0U;
    vehicle->arm_hold_ms = arm_hold_ms;
    vehicle->last_rc_ms = 0U;
    vehicle->rc_timeout_ms = rc_timeout_ms;
    vehicle->state_changes = 0U;
    return FATTOS_OK;
}

void fattos_vehicle_set_rc(FattosVehicle *vehicle, bool present,
                           bool arm_switch)
{
    if (vehicle == NULL) {
        return;
    }
    vehicle->rc_present = present;
    vehicle->rc_arm_switch = arm_switch;
    if (present) {
        vehicle->last_rc_ms = fattos_millis();
    }
}

void fattos_vehicle_set_prearm(FattosVehicle *vehicle, bool ok)
{
    if (vehicle != NULL) {
        vehicle->prearm_ok = ok;
    }
}

static void enter_state(FattosVehicle *vehicle, FattosVehicleState state)
{
    if (vehicle->state != state) {
        vehicle->state = state;
        vehicle->state_changes++;
        vehicle->arm_pending = false;
    }
}

FattosVehicleState fattos_vehicle_update(FattosVehicle *vehicle)
{
    if (vehicle == NULL) {
        return FATTOS_VEHICLE_INIT;
    }
    uint32_t now = fattos_millis();

    bool rc_ok = vehicle->rc_present &&
                 (int32_t)(now - (vehicle->last_rc_ms + vehicle->rc_timeout_ms)) < 0;

    bool supervisor_estop = false;
    bool supervisor_fault = false;
    bool supervisor_ok = true;
    if (vehicle->safety != NULL) {
        /* SAFETY: evaluate the supervisor freshly here (timeouts + latch),
           never trust the last polled state — the gate must react within
           one control cycle even if nobody else polled. As a bonus the
           poll drives zero duty on any fault. */
        FattosSafetyState ss = fattos_safety_poll(vehicle->safety);
        supervisor_estop = (ss == FATTOS_SAFETY_ESTOP_LATCHED);
        supervisor_fault = (ss == FATTOS_SAFETY_HEARTBEAT_TIMEOUT);
        supervisor_ok = (ss == FATTOS_SAFETY_OK);
    }

    /* Sensor-manifest health joins the gate automatically: if the board
       declared sensors, every one of them must be present, fresh and
       plausible to arm — and a sensor that dies in flight drops the
       vehicle to DISARMED via the same prearm bit. No manifest declared?
       Then the application's own prearm bit is the whole story. */
    bool health_ok = vehicle->prearm_ok;
    if (fattos_sensor_count() > 0U) {
        health_ok = health_ok && fattos_sensors_prearm_ok();
    }

    bool arm_conditions = rc_ok && vehicle->rc_arm_switch &&
                          health_ok && supervisor_ok;

    switch (vehicle->state) {
    case FATTOS_VEHICLE_INIT:
        enter_state(vehicle, FATTOS_VEHICLE_DISARMED);
        break;

    case FATTOS_VEHICLE_DISARMED:
        if (arm_conditions) {
            /* Deliberate arming: switch held for arm_hold_ms, all healthy.
               (Explicit pending flag: millis()==0 is a valid anchor.) */
            if (!vehicle->arm_pending) {
                vehicle->arm_pending = true;
                vehicle->arm_request_ms = now;
            } else if ((uint32_t)(now - vehicle->arm_request_ms) >=
                       vehicle->arm_hold_ms) {
                enter_state(vehicle, FATTOS_VEHICLE_ARMED);
            }
        } else {
            vehicle->arm_pending = false;
        }
        break;

    case FATTOS_VEHICLE_ARMED:
        if (supervisor_estop) {
            enter_state(vehicle, FATTOS_VEHICLE_ESTOP);
        } else if (!rc_ok || supervisor_fault || !health_ok) {
            enter_state(vehicle, FATTOS_VEHICLE_FAILSAFE);
        } else if (!vehicle->rc_arm_switch) {
            enter_state(vehicle, FATTOS_VEHICLE_DISARMED); /* pilot disarm */
        }
        break;

    case FATTOS_VEHICLE_FAILSAFE:
        if (supervisor_estop) {
            enter_state(vehicle, FATTOS_VEHICLE_ESTOP);
        } else if (rc_ok && vehicle->rc_arm_switch && supervisor_ok) {
            /* Recovery requires a full re-arm cycle, never a silent jump. */
            enter_state(vehicle, FATTOS_VEHICLE_DISARMED);
        }
        break;

    case FATTOS_VEHICLE_ESTOP:
        if (supervisor_ok && !supervisor_estop) {
            /* Only reachable after an explicit estop_clear. */
            enter_state(vehicle, FATTOS_VEHICLE_DISARMED);
        }
        break;

    default:
        enter_state(vehicle, FATTOS_VEHICLE_FAILSAFE);
        break;
    }
    return vehicle->state;
}

bool fattos_vehicle_can_actuate(const FattosVehicle *vehicle)
{
    if (vehicle == NULL || vehicle->state != FATTOS_VEHICLE_ARMED) {
        return false;
    }
    return vehicle->safety == NULL || fattos_safety_is_safe(vehicle->safety);
}

FattosStatus fattos_vehicle_estop(FattosVehicle *vehicle)
{
    if (vehicle == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    FattosStatus status = FATTOS_OK;
    if (vehicle->safety != NULL) {
        status = fattos_safety_estop(vehicle->safety);
    }
    enter_state(vehicle, FATTOS_VEHICLE_ESTOP);
    return status;
}

FattosStatus fattos_vehicle_estop_clear(FattosVehicle *vehicle)
{
    if (vehicle == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    /* Refused while the supervisor is still faulted — recovery must be
       earned: heartbeats healthy, no timeouts. */
    if (vehicle->safety != NULL) {
        FattosStatus status = fattos_safety_estop_clear(vehicle->safety);
        if (status != FATTOS_OK) {
            return status;
        }
    }
    enter_state(vehicle, FATTOS_VEHICLE_DISARMED);
    return FATTOS_OK;
}