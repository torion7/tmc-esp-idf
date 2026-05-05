#pragma once

// Adapter binding the TMC5240 chip driver into tmc_motion's driver-ops
// interface. Include this header and pass &tmc_motion_ops_5240 in the
// tmc_motion_config_t to use a TMC5240 with tmc_motion.

#include "tmc_motion_driver.h"
#include "tmc5240.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const tmc_motion_driver_ops_t tmc_motion_ops_5240;

#ifdef __cplusplus
}
#endif
