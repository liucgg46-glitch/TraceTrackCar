#include "drv_vl53l1x.h"

#if DRV_VL53L1X_ENABLE

#include "VL53L1X_api.h"
#include "vl53l1_platform_config.h"
#include "bsp_systick.h"
#include <string.h>

static Drv_VL53L1X_Info_t s_tof;
static VL53L1X_Result_t s_pending_result;
static uint32_t s_state_enter_ms;
static uint32_t s_last_action_ms;
static uint8_t s_reinit_requested;
static uint32_t s_poll_delay_ms;

static void VL53_StateEnter(Drv_VL53L1X_State_t state)
{
    s_tof.state = state;
    s_state_enter_ms = BSP_GET_TICK();
}

static uint8_t VL53_TimeReached(uint32_t start_ms, uint32_t wait_ms)
{
    return ((uint32_t)(BSP_GET_TICK() - start_ms) >= wait_ms) ? 1U : 0U;
}

static uint8_t VL53_BusAvailable(void)
{
    if (BSP_I2C_IsBusy(DRV_VL53L1X_I2C_BUS) != 0U) {
        s_tof.busy_skip_count++;
        return 0U;
    }
    return 1U;
}

static void VL53_RecordApiSuccess(void)
{
    s_tof.last_api_status = VL53L1X_ERROR_NONE;
    s_tof.consecutive_error_count = 0U;
}

static void VL53_EnterErrorWait(uint8_t api_status)
{
    s_tof.last_api_status = api_status;
    s_tof.online = 0U;
    s_tof.initialized = 0U;
    s_tof.ranging = 0U;
    s_tof.data_ready = 0U;
    VL53_StateEnter(DRV_VL53L1X_STATE_ERROR_WAIT);
}

static void VL53_RecordInitError(uint8_t api_status)
{
    s_tof.last_api_status = api_status;
    s_tof.error_count++;
    if (s_tof.consecutive_error_count < 0xFFU) {
        s_tof.consecutive_error_count++;
    }
    VL53_EnterErrorWait(api_status);
}

static void VL53_RecordRuntimeError(uint8_t api_status)
{
    s_tof.last_api_status = api_status;
    s_tof.error_count++;
    if (s_tof.consecutive_error_count < 0xFFU) {
        s_tof.consecutive_error_count++;
    }

    s_tof.data_ready = 0U;
    if (s_tof.consecutive_error_count >= DRV_VL53L1X_MAX_CONSECUTIVE_ERRORS) {
        VL53_EnterErrorWait(api_status);
    } else {
        s_last_action_ms = BSP_GET_TICK();
        s_poll_delay_ms = DRV_VL53L1X_RUNTIME_RETRY_DELAY_MS;
        VL53_StateEnter(DRV_VL53L1X_STATE_CHECK_DATA_READY);
    }
}

static uint16_t VL53_FilterDistance(uint16_t previous, uint16_t current)
{
#if DRV_VL53L1X_FILTER_ENABLE
    uint32_t sum;

    if ((s_tof.valid_count == 0U) || (DRV_VL53L1X_FILTER_DIV <= 1U)) {
        return current;
    }

    sum = (uint32_t)previous * (DRV_VL53L1X_FILTER_DIV - 1U);
    sum += current;
    return (uint16_t)(sum / DRV_VL53L1X_FILTER_DIV);
#else
    (void)previous;
    return current;
#endif
}

static void VL53_PublishResult(void)
{
    uint8_t valid;
    uint32_t now = BSP_GET_TICK();

    s_tof.range_status = s_pending_result.Status;
    s_tof.raw_distance_mm = s_pending_result.Distance;
    s_tof.ambient_kcps = s_pending_result.Ambient;
    s_tof.signal_per_spad_kcps = s_pending_result.SigPerSPAD;
    s_tof.spad_count = s_pending_result.NumSPADs;
    s_tof.measurement_count++;
    s_tof.last_sample_ms = now;
    s_tof.online = 1U;
    s_tof.new_data = 1U;

    valid = 1U;
    if (s_tof.range_status != DRV_VL53L1X_VALID_RANGE_STATUS) {
        valid = 0U;
    }
    if ((s_tof.raw_distance_mm < DRV_VL53L1X_MIN_VALID_DISTANCE_MM) ||
        (s_tof.raw_distance_mm > DRV_VL53L1X_MAX_VALID_DISTANCE_MM)) {
        valid = 0U;
    }

    s_tof.data_valid = valid;
    if (valid != 0U) {
        s_tof.distance_mm = VL53_FilterDistance(s_tof.distance_mm,
                                                s_tof.raw_distance_mm);
        s_tof.valid_count++;
        s_tof.last_valid_ms = now;
    }
}

static void VL53_StartBootSequence(void)
{
    s_tof.boot_state = 0U;
    s_tof.data_ready = 0U;
    s_tof.initialized = 0U;
    s_tof.ranging = 0U;
    s_tof.online = 0U;
    s_tof.data_valid = 0U;

#if DRV_VL53L1X_USE_XSHUT
    BSP_GPIO_Write(DRV_VL53L1X_XSHUT_GPIO, 0U);
    VL53_StateEnter(DRV_VL53L1X_STATE_XSHUT_LOW);
#else
    VL53_StateEnter(DRV_VL53L1X_STATE_POWER_ON_WAIT);
#endif
}

void Drv_VL53L1X_Init(void)
{
    memset(&s_tof, 0, sizeof(s_tof));
    memset(&s_pending_result, 0, sizeof(s_pending_result));

    s_tof.enabled = 1U;
    s_tof.state = DRV_VL53L1X_STATE_DISABLED;
    s_last_action_ms = BSP_GET_TICK();
    s_reinit_requested = 0U;
    s_poll_delay_ms = DRV_VL53L1X_READY_POLL_PERIOD_MS;

    VL53_StartBootSequence();
}

BSP_Status_t Drv_VL53L1X_Update(void)
{
    VL53L1X_ERROR api_status;
    uint8_t ready;
    uint16_t sensor_id = 0U;
    uint32_t now;

    if (s_tof.enabled == 0U) {
        return BSP_ERROR;
    }

    now = BSP_GET_TICK();

    if (s_reinit_requested != 0U) {
        s_reinit_requested = 0U;
        s_tof.reinit_count++;
        VL53_StartBootSequence();
        return BSP_OK;
    }

    if ((s_tof.ranging != 0U) && (s_tof.last_sample_ms != 0U) &&
        ((uint32_t)(now - s_tof.last_sample_ms) > DRV_VL53L1X_ONLINE_TIMEOUT_MS)) {
        VL53_RecordInitError(VL53L1X_ERROR_TIMEOUT);
        return BSP_TIMEOUT;
    }

    switch (s_tof.state) {
        case DRV_VL53L1X_STATE_DISABLED:
            return BSP_ERROR;

        case DRV_VL53L1X_STATE_XSHUT_LOW:
#if DRV_VL53L1X_USE_XSHUT
            if (VL53_TimeReached(s_state_enter_ms, DRV_VL53L1X_XSHUT_LOW_MS) == 0U) {
                return BSP_BUSY;
            }
            BSP_GPIO_Write(DRV_VL53L1X_XSHUT_GPIO, 1U);
            VL53_StateEnter(DRV_VL53L1X_STATE_POWER_ON_WAIT);
            return BSP_OK;
#else
            VL53_StateEnter(DRV_VL53L1X_STATE_POWER_ON_WAIT);
            return BSP_OK;
#endif

        case DRV_VL53L1X_STATE_POWER_ON_WAIT:
            if (VL53_TimeReached(s_state_enter_ms, DRV_VL53L1X_POWER_ON_WAIT_MS) == 0U) {
                return BSP_BUSY;
            }
            s_last_action_ms = now;
            VL53_StateEnter(DRV_VL53L1X_STATE_BOOT_CHECK);
            return BSP_OK;

        case DRV_VL53L1X_STATE_BOOT_CHECK:
            if ((uint32_t)(now - s_state_enter_ms) > DRV_VL53L1X_BOOT_TIMEOUT_MS) {
                VL53_RecordInitError(VL53L1X_ERROR_TIMEOUT);
                return BSP_TIMEOUT;
            }
            if (VL53_TimeReached(s_last_action_ms, DRV_VL53L1X_BOOT_POLL_PERIOD_MS) == 0U) {
                return BSP_BUSY;
            }
            if (VL53_BusAvailable() == 0U) {
                return BSP_BUSY;
            }
            s_last_action_ms = now;
            ready = 0U;
            api_status = VL53L1X_BootState(DRV_VL53L1X_I2C_ADDR_8BIT, &ready);
            s_tof.boot_state = ready;
            if (api_status != VL53L1X_ERROR_NONE) {
                VL53_RecordInitError(api_status);
                return BSP_ERROR;
            }
            VL53_RecordApiSuccess();
            if (ready != 0U) {
                VL53_StateEnter(DRV_VL53L1X_STATE_SENSOR_INIT);
            }
            return BSP_OK;

        case DRV_VL53L1X_STATE_SENSOR_INIT:
            if (VL53_BusAvailable() == 0U) {
                return BSP_BUSY;
            }
            api_status = VL53L1X_GetSensorId(DRV_VL53L1X_I2C_ADDR_8BIT, &sensor_id);
            /* 先保存实际读到的 ID，后续即使校验失败，测试日志也能直接显示原始值。 */
            s_tof.sensor_id = sensor_id;

            if (api_status != VL53L1X_ERROR_NONE) {
                VL53_RecordInitError(api_status);
                return BSP_ERROR;
            }
            if (sensor_id != DRV_VL53L1X_EXPECTED_SENSOR_ID) {
                VL53_RecordInitError(VL53L1X_ERROR_INVALID_ARGUMENT);
                return BSP_ERROR;
            }

            api_status = VL53L1X_SensorInit(DRV_VL53L1X_I2C_ADDR_8BIT);
            if (api_status != VL53L1X_ERROR_NONE) {
                VL53_RecordInitError(api_status);
                return BSP_ERROR;
            }
            VL53_RecordApiSuccess();
            s_tof.initialized = 1U;
            VL53_StateEnter(DRV_VL53L1X_STATE_SET_DISTANCE_MODE);
            return BSP_OK;

        case DRV_VL53L1X_STATE_SET_DISTANCE_MODE:
            if (VL53_BusAvailable() == 0U) return BSP_BUSY;
            api_status = VL53L1X_SetDistanceMode(DRV_VL53L1X_I2C_ADDR_8BIT,
                                                 DRV_VL53L1X_DISTANCE_MODE);
            if (api_status != 0U) {
                VL53_RecordInitError(api_status);
                return BSP_ERROR;
            }
            VL53_RecordApiSuccess();
            VL53_StateEnter(DRV_VL53L1X_STATE_SET_TIMING_BUDGET);
            return BSP_OK;

        case DRV_VL53L1X_STATE_SET_TIMING_BUDGET:
            if (VL53_BusAvailable() == 0U) return BSP_BUSY;
            api_status = VL53L1X_SetTimingBudgetInMs(DRV_VL53L1X_I2C_ADDR_8BIT,
                                                     DRV_VL53L1X_TIMING_BUDGET_MS);
            if (api_status != 0U) {
                VL53_RecordInitError(api_status);
                return BSP_ERROR;
            }
            VL53_RecordApiSuccess();
            VL53_StateEnter(DRV_VL53L1X_STATE_SET_INTER_MEASUREMENT);
            return BSP_OK;

        case DRV_VL53L1X_STATE_SET_INTER_MEASUREMENT:
            if (VL53_BusAvailable() == 0U) return BSP_BUSY;
            api_status = VL53L1X_SetInterMeasurementInMs(DRV_VL53L1X_I2C_ADDR_8BIT,
                                                         DRV_VL53L1X_INTER_MEASUREMENT_MS);
            if (api_status != 0U) {
                VL53_RecordInitError(api_status);
                return BSP_ERROR;
            }
            VL53_RecordApiSuccess();
            VL53_StateEnter(DRV_VL53L1X_STATE_SET_INTERRUPT_POLARITY);
            return BSP_OK;

        case DRV_VL53L1X_STATE_SET_INTERRUPT_POLARITY:
            if (VL53_BusAvailable() == 0U) return BSP_BUSY;
            api_status = VL53L1X_SetInterruptPolarity(DRV_VL53L1X_I2C_ADDR_8BIT,
                                                      DRV_VL53L1X_INTERRUPT_ACTIVE_HIGH);
            if (api_status != 0U) {
                VL53_RecordInitError(api_status);
                return BSP_ERROR;
            }
            VL53_RecordApiSuccess();
#if DRV_VL53L1X_APPLY_OFFSET_ENABLE
            VL53_StateEnter(DRV_VL53L1X_STATE_SET_OFFSET);
#elif DRV_VL53L1X_APPLY_XTALK_ENABLE
            VL53_StateEnter(DRV_VL53L1X_STATE_SET_XTALK);
#else
            VL53_StateEnter(DRV_VL53L1X_STATE_START_RANGING);
#endif
            return BSP_OK;

        case DRV_VL53L1X_STATE_SET_OFFSET:
#if DRV_VL53L1X_APPLY_OFFSET_ENABLE
            if (VL53_BusAvailable() == 0U) return BSP_BUSY;
            api_status = VL53L1X_SetOffset(DRV_VL53L1X_I2C_ADDR_8BIT,
                                           (int16_t)DRV_VL53L1X_OFFSET_MM);
            if (api_status != 0U) {
                VL53_RecordInitError(api_status);
                return BSP_ERROR;
            }
            VL53_RecordApiSuccess();
#endif
#if DRV_VL53L1X_APPLY_XTALK_ENABLE
            VL53_StateEnter(DRV_VL53L1X_STATE_SET_XTALK);
#else
            VL53_StateEnter(DRV_VL53L1X_STATE_START_RANGING);
#endif
            return BSP_OK;

        case DRV_VL53L1X_STATE_SET_XTALK:
#if DRV_VL53L1X_APPLY_XTALK_ENABLE
            if (VL53_BusAvailable() == 0U) return BSP_BUSY;
            api_status = VL53L1X_SetXtalk(DRV_VL53L1X_I2C_ADDR_8BIT,
                                          DRV_VL53L1X_XTALK_CPS);
            if (api_status != 0U) {
                VL53_RecordInitError(api_status);
                return BSP_ERROR;
            }
            VL53_RecordApiSuccess();
#endif
            VL53_StateEnter(DRV_VL53L1X_STATE_START_RANGING);
            return BSP_OK;

        case DRV_VL53L1X_STATE_START_RANGING:
            if (VL53_BusAvailable() == 0U) return BSP_BUSY;
            api_status = VL53L1X_StartRanging(DRV_VL53L1X_I2C_ADDR_8BIT);
            if (api_status != 0U) {
                VL53_RecordInitError(api_status);
                return BSP_ERROR;
            }
            VL53_RecordApiSuccess();
            s_tof.ranging = 1U;
            s_tof.online = 1U;
            s_tof.last_sample_ms = now;
            s_last_action_ms = now;
            s_poll_delay_ms = DRV_VL53L1X_READY_POLL_PERIOD_MS;
            VL53_StateEnter(DRV_VL53L1X_STATE_CHECK_DATA_READY);
            return BSP_OK;

        case DRV_VL53L1X_STATE_CHECK_DATA_READY:
            if (VL53_TimeReached(s_last_action_ms, s_poll_delay_ms) == 0U) {
                return BSP_BUSY;
            }
            if (VL53_BusAvailable() == 0U) return BSP_BUSY;
            s_last_action_ms = now;
            ready = 0U;
            api_status = VL53L1X_CheckForDataReady(DRV_VL53L1X_I2C_ADDR_8BIT, &ready);
            if (api_status != 0U) {
                VL53_RecordRuntimeError(api_status);
                return BSP_ERROR;
            }
            s_tof.last_api_status = VL53L1X_ERROR_NONE;
            s_poll_delay_ms = DRV_VL53L1X_READY_POLL_PERIOD_MS;
            s_tof.data_ready = ready;
            if (ready != 0U) {
                VL53_StateEnter(DRV_VL53L1X_STATE_READ_RESULT);
            }
            return BSP_OK;

        case DRV_VL53L1X_STATE_READ_RESULT:
            if (VL53_BusAvailable() == 0U) return BSP_BUSY;
            api_status = VL53L1X_GetResult(DRV_VL53L1X_I2C_ADDR_8BIT,
                                           &s_pending_result);
            if (api_status != 0U) {
                VL53_RecordRuntimeError(api_status);
                return BSP_ERROR;
            }
            s_tof.last_api_status = VL53L1X_ERROR_NONE;
            VL53_StateEnter(DRV_VL53L1X_STATE_CLEAR_INTERRUPT);
            return BSP_OK;

        case DRV_VL53L1X_STATE_CLEAR_INTERRUPT:
            if (VL53_BusAvailable() == 0U) return BSP_BUSY;
            api_status = VL53L1X_ClearInterrupt(DRV_VL53L1X_I2C_ADDR_8BIT);
            if (api_status != 0U) {
                VL53_RecordRuntimeError(api_status);
                return BSP_ERROR;
            }
            VL53_RecordApiSuccess();
            s_tof.data_ready = 0U;
            VL53_PublishResult();
            s_last_action_ms = now;
            s_poll_delay_ms = DRV_VL53L1X_READY_POLL_PERIOD_MS;
            VL53_StateEnter(DRV_VL53L1X_STATE_CHECK_DATA_READY);
            return BSP_OK;

        case DRV_VL53L1X_STATE_ERROR_WAIT:
            if (VL53_TimeReached(s_state_enter_ms, DRV_VL53L1X_REINIT_DELAY_MS) == 0U) {
                return BSP_BUSY;
            }
            s_tof.reinit_count++;
            VL53_StartBootSequence();
            return BSP_OK;

        case DRV_VL53L1X_STATE_STOPPED:
            return BSP_OK;

        default:
            VL53_RecordInitError(VL53L1X_ERROR_INVALID_ARGUMENT);
            return BSP_ERROR;
    }
}

void Drv_VL53L1X_RequestReinit(void)
{
    s_reinit_requested = 1U;
}

BSP_Status_t Drv_VL53L1X_Stop(void)
{
    VL53L1X_ERROR api_status;

    if (s_tof.enabled == 0U) {
        return BSP_ERROR;
    }
    if (BSP_I2C_IsBusy(DRV_VL53L1X_I2C_BUS) != 0U) {
        return BSP_BUSY;
    }

    if (s_tof.ranging != 0U) {
        api_status = VL53L1X_StopRanging(DRV_VL53L1X_I2C_ADDR_8BIT);
        if (api_status != 0U) {
            s_tof.last_api_status = api_status;
            s_tof.error_count++;
            return BSP_ERROR;
        }
    }

    s_tof.ranging = 0U;
    s_tof.online = 0U;
    VL53_StateEnter(DRV_VL53L1X_STATE_STOPPED);
    return BSP_OK;
}

uint8_t Drv_VL53L1X_IsOnline(void)
{
    return s_tof.online;
}

uint8_t Drv_VL53L1X_HasNewData(void)
{
    return s_tof.new_data;
}

void Drv_VL53L1X_ClearNewData(void)
{
    s_tof.new_data = 0U;
}

BSP_Status_t Drv_VL53L1X_GetDistanceMm(uint16_t *distance_mm)
{
    if (distance_mm == 0) {
        return BSP_PARAM;
    }
    if ((s_tof.online == 0U) || (s_tof.data_valid == 0U)) {
        return BSP_ERROR;
    }

    *distance_mm = s_tof.distance_mm;
    return BSP_OK;
}

BSP_Status_t Drv_VL53L1X_GetInfo(Drv_VL53L1X_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }

    *info = s_tof;
    return BSP_OK;
}

#else

#include <string.h>

void Drv_VL53L1X_Init(void) {}
BSP_Status_t Drv_VL53L1X_Update(void) { return BSP_ERROR; }
void Drv_VL53L1X_RequestReinit(void) {}
BSP_Status_t Drv_VL53L1X_Stop(void) { return BSP_ERROR; }
uint8_t Drv_VL53L1X_IsOnline(void) { return 0U; }
uint8_t Drv_VL53L1X_HasNewData(void) { return 0U; }
void Drv_VL53L1X_ClearNewData(void) {}
BSP_Status_t Drv_VL53L1X_GetDistanceMm(uint16_t *distance_mm)
{
    (void)distance_mm;
    return BSP_ERROR;
}
BSP_Status_t Drv_VL53L1X_GetInfo(Drv_VL53L1X_Info_t *info)
{
    if (info != 0) memset(info, 0, sizeof(*info));
    return BSP_ERROR;
}

#endif /* DRV_VL53L1X_ENABLE */
