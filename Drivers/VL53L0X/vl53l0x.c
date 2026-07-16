#include "vl53l0x.h"

#include "clock.h"
#include "vl53l0x_api.h"

#define VL53L0X_DEFAULT_TIMING_BUDGET_US (33000UL)
#define VL53L0X_MODEL_ID                 (0xEEAAU)

static VL53L0X_Dev_t sensor = {
    .Id = 0,
    .I2cDevAddr = VL53L0X_I2C_ADDRESS_7BIT,
    .Present = 1,
};

static VL53L0X_RangingMeasurementData_t measurement;
static uint16_t latest_distance_mm;
static bool initialized;
static bool distance_valid;

#define VL53L0X_TRY(expression)                         \
    do {                                                \
        if ((expression) != VL53L0X_ERROR_NONE) {       \
            initialized = false;                        \
            distance_valid = false;                     \
            return false;                               \
        }                                               \
    } while (0)

bool VL53L0X_Init(void)
{
    uint16_t model_id;
    uint8_t vhv_settings;
    uint8_t phase_cal;
    uint32_t ref_spad_count;
    uint8_t is_aperture_spads;

    initialized = false;
    distance_valid = false;

    /* The GY-VL53L0XV2 board pulls XSHUT high, so only boot time is needed. */
    mspm0_delay_ms(3U);

    VL53L0X_TRY(VL53L0X_RdWord(
        &sensor, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id));
    if (model_id != VL53L0X_MODEL_ID) {
        return false;
    }

    VL53L0X_TRY(VL53L0X_DataInit(&sensor));
    VL53L0X_TRY(VL53L0X_StaticInit(&sensor));
    VL53L0X_TRY(VL53L0X_PerformRefSpadManagement(
        &sensor, &ref_spad_count, &is_aperture_spads));
    VL53L0X_TRY(VL53L0X_PerformRefCalibration(
        &sensor, &vhv_settings, &phase_cal));
    VL53L0X_TRY(VL53L0X_SetDeviceMode(
        &sensor, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING));
    VL53L0X_TRY(VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
        &sensor, VL53L0X_DEFAULT_TIMING_BUDGET_US));
    VL53L0X_TRY(VL53L0X_ClearInterruptMask(&sensor, 0U));
    VL53L0X_TRY(VL53L0X_StartMeasurement(&sensor));

    initialized = true;
    return true;
}

bool VL53L0X_Process(void)
{
    uint8_t measurement_ready = 0U;

    if (!initialized) {
        return false;
    }

    if (VL53L0X_GetMeasurementDataReady(&sensor, &measurement_ready) !=
        VL53L0X_ERROR_NONE) {
        initialized = false;
        distance_valid = false;
        return false;
    }
    if (measurement_ready == 0U) {
        return false;
    }

    if (VL53L0X_GetRangingMeasurementData(&sensor, &measurement) !=
        VL53L0X_ERROR_NONE) {
        initialized = false;
        distance_valid = false;
        return false;
    }
    (void)VL53L0X_ClearInterruptMask(&sensor, 0U);

    distance_valid = (measurement.RangeStatus == 0U);
    if (distance_valid) {
        latest_distance_mm = measurement.RangeMilliMeter;
    }
    return distance_valid;
}

bool VL53L0X_GetDistance(uint16_t *distance_mm)
{
    if ((distance_mm == NULL) || !distance_valid) {
        return false;
    }

    *distance_mm = latest_distance_mm;
    return true;
}
