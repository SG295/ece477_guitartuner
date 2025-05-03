Import("env")

# Add DSP library source files
dsp_dir = "${PLATFORMIO_PACKAGES_DIR}/framework-stm32cubef4/Drivers/CMSIS/DSP/Source"
dsp_sources = [
    dsp_dir + "/BasicMathFunctions/arm_add_f32.c",
    dsp_dir + "/BasicMathFunctions/arm_dot_prod_f32.c",
    dsp_dir + "/BasicMathFunctions/arm_mult_f32.c",
    dsp_dir + "/BasicMathFunctions/arm_scale_f32.c",
    dsp_dir + "/BasicMathFunctions/arm_sub_f32.c",
    dsp_dir + "/CommonTables/arm_common_tables.c",
    dsp_dir + "/CommonTables/arm_const_structs.c",
    dsp_dir + "/ComplexMathFunctions/arm_cmplx_mag_f32.c",
    dsp_dir + "/TransformFunctions/arm_rfft_fast_f32.c",
    dsp_dir + "/TransformFunctions/arm_rfft_fast_init_f32.c",
    dsp_dir + "/TransformFunctions/arm_cfft_f32.c",
    dsp_dir + "/TransformFunctions/arm_cfft_radix8_f32.c",
    dsp_dir + "/TransformFunctions/arm_bitreversal.c",
    dsp_dir + "/TransformFunctions/arm_bitreversal2.S",
]

env.Append(
    CPPDEFINES=[
        "ARM_MATH_CM4",
        "__FPU_PRESENT=1"
    ]
)

for source in dsp_sources:
    env.BuildSources(
        "${BUILD_DIR}/CMSIS_DSP",
        dsp_dir,
        "+<" + source.replace(dsp_dir + "/", "") + ">"
    )