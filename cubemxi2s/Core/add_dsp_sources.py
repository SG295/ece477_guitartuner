from os.path import join
Import("env")

# Get the path to the framework directory
FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-stm32cubef4")

# DSP library source files needed for FIR filter
dsp_src_files = [
    FRAMEWORK_DIR + "/Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_f32.c",
    FRAMEWORK_DIR + "/Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_f32.c",
    FRAMEWORK_DIR + "/Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c",
    FRAMEWORK_DIR + "/Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.c",
    FRAMEWORK_DIR + "/Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.c",
    FRAMEWORK_DIR + "/Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.c",
    FRAMEWORK_DIR + "/Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.c"
]

# Add the files to the build environment
env.Append(
    CPPPATH=[
        FRAMEWORK_DIR + "/Drivers/CMSIS/DSP/Include",
    ]
)

env.Append(
    LIBS=[
        env.BuildLibrary(
            join("$BUILD_DIR", "CMSIS_DSP"),
            FRAMEWORK_DIR + "/Drivers/CMSIS/DSP/Source",
            src_filter="+<FilteringFunctions/arm_fir_f32.c> +<FilteringFunctions/arm_fir_init_f32.c> +<CommonTables/arm_common_tables.c> +<SupportFunctions/arm_float_to_q31.c> +<SupportFunctions/arm_float_to_q15.c> +<SupportFunctions/arm_q31_to_float.c> +<SupportFunctions/arm_q15_to_float.c>"
        )
    ]
)