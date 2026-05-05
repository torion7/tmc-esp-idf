idf_component_register(
    SRCS
        "src/tmc_motion.c"
        "src/tmc_motion_units.c"
        "src/tmc_motion_motion.c"
        "src/tmc_motion_stallguard.c"
        "src/adapters/adapter_5240.c"
        "src/adapters/adapter_5160.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES
        tmc5240
        tmc5160
        esp_timer
)
