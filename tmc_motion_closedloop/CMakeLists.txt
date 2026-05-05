idf_component_register(
    SRCS
        "src/tmc_motion_closedloop.c"
        "src/encoder_5240.c"
    INCLUDE_DIRS "include"
    REQUIRES
        tmc_motion
        tmc5240
        esp_timer
)
