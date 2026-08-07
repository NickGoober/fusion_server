idf_component_register(SRCS "fusion.c"
                            "cf/arm_math_shim.c"
                            "cf/kalman_core.c"
                            "cf/kalman_supervisor.c"
                            "cf/mm_flow.c"
                            "cf/mm_tof.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "cf"
                       PRIV_REQUIRES esp_timer)

# The vendored Crazyflie sources (components/Fusion/cf) are GPLv3 (Bitcraze AB
# and contributors); math3d.h is MIT (James Alan Preiss). Headers retained.
#
# -fno-strict-aliasing: math3d.h's fcloseulps() type-puns floats (upstream
#   builds the same way); -Wno-absolute-value: same helper calls fabsf on int.
target_compile_options(${COMPONENT_LIB} PRIVATE
                       -Wno-missing-field-initializers
                       -Wno-absolute-value
                       -fno-strict-aliasing)
