
#ifndef __STEPPER_DRIVER_H_
#define __STEPPER_DRIVER_H_


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "stm32f407xx.h"
#include <stdint.h>
#include <math.h>

/* ------------------------------------------------------------------ */
#ifdef USE_HAL_DRIVER

	#include "main.h"

	/* --------------- Check Mainstream series --------------- */

	#ifdef STM32F0
		#include "stm32f0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F1)
		#include "stm32f1xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F2)
		#include "stm32f2xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F3)
		#include "stm32f3xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F4)
		#include "stm32f4xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F7)
		#include "stm32f7xx_hal.h"       /* Import HAL library */
	#elif defined(STM32G0)
		#include "stm32g0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32G4)
		#include "stm32g4xx_hal.h"       /* Import HAL library */

	/* ------------ Check High Performance series ------------ */

	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */

	/* ------------ Check Ultra low power series ------------- */

	#elif defined(STM32L0)
		#include "stm32l0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L1)
		#include "stm32l1xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L5)
		#include "stm32l5xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L4)
		#include "stm32l4xx_hal.h"       /* Import HAL library */
	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
	#else
	#endif /* STM32F1 */

	/* ------------------------------------------------------- */

	#ifdef __ICCARM__ /* ICCARM Compiler */

	#pragma diag_suppress=Pe177   /* Disable 'unused function' warning */

	#endif /* __ICCARM__ */

/* ------------------------------------------------------------------ */

#else                     /* Compiler not found */

#error Chip or Library not supported  /* Send error */

#endif /* USE_HAL_DRIVER */

/* ------------------------------------------------------------------ */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ---------------------------- Public ---------------------------- */
/* ---------------------- Define by compiler ---------------------- */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
typedef void (*StepperRun_T)(uint16_t step);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
class Stepper
{

	private:

	/* ~~~~~~~~~~~~~~~~~~~~~~~~~ Variable ~~~~~~~~~~~~~~~~~~~~~~~~~ */
	/* ::::::::::::: Stepper param :::::::::::: */
	uint16_t       _numberOfSteps;
	uint32_t       _stepDelay;

	/* :::::::::::::: Calculation ::::::::::::: */
	float          _angRatio;

	/* ::::::::::::: Stepper GPIO ::::::::::::: */
	GPIO_TypeDef  *_enGPIO;
	GPIO_TypeDef  *_dirGPIO;
	GPIO_TypeDef  *_clkGPIO;

	uint16_t       _enPIN;
	uint16_t       _dirPIN;
	uint16_t       _clkPIN;

	public:

	/* ~~~~~~~~~~~~~~~~~~~~~~~~ Prototupe ~~~~~~~~~~~~~~~~~~~~~~~~~ */
	/* :::::::::::::: Constructor ::::::::::::: */
	Stepper();
	Stepper(GPIO_TypeDef *enGPIO, uint32_t enPIN, GPIO_TypeDef *dirGPIO, uint32_t dirPIN, GPIO_TypeDef *clkGPIO, uint32_t clkPIN, uint16_t  numberOfSteps = 200, uint16_t  rpm = 50);

	/* ::::::::::::::::: Init ::::::::::::::::: */
	void Init(GPIO_TypeDef *enGPIO, uint32_t enPIN, GPIO_TypeDef *dirGPIO, uint32_t dirPIN, GPIO_TypeDef *clkGPIO, uint32_t clkPIN, uint16_t  numberOfSteps = 200, uint16_t  rpm = 50);
	void SetSpeed(uint16_t rpm);

	/* ::::::::::::::: Position ::::::::::::::: */
	void Step(int16_t step);
	void Step(int16_t step, StepperRun_T func);

	void StepAngle(float ang);
	void StepAngle(float ang, StepperRun_T func);

};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#endif /* __STEPPER_DRIVER_H_ */