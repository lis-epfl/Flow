/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include <px4_config.h>
#include <bsp/board.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include "no_warnings.h"
#include "mavlink_bridge_header.h"
#include <mavlink.h>
// #include "mavlink/v1.0/mavric/mavlink.h"
#include "settings.h"
#include "utils.h"
#include "led.h"
#include "flow.h"
#include "dcmi.h"
#include "mt9v034.h"
#include "gyro.h"
#include "i2c.h"
#include "usart.h"
#include "sonar.h"
#include "communication.h"
#include "debug.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "main.h"
#include <uavcan_if.h>
#include <px4_macros.h>

//#define CONFIG_USE_PROBES
#include <bsp/probes.h>

// #include "lucaskanade.h"
#include "maths.h"
// #include "cam.h"
#include "flow2.h"
#include "../calib/calib_cam3.h"
// #include "emd.h"
#include "emdarray.h"

/* coprocessor control register (fpu) */
#ifndef SCB_CPACR
#define SCB_CPACR (*((uint32_t*) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif



/* prototypes */
void delay(unsigned msec);
void buffer_reset(void);

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

/* fast image buffers for calculations */
// uint8_t image_buffer_8bit_1[FULL_IMAGE_SIZE] __attribute__((section(".ccm")));
// uint8_t image_buffer_8bit_2[FULL_IMAGE_SIZE] __attribute__((section(".ccm")));
uint8_t image_buffer_8bit_1[BOTTOM_FLOW_IMAGE_WIDTH * BOTTOM_FLOW_IMAGE_HEIGHT] __attribute__((section(".ccm")));
uint8_t image_buffer_8bit_2[BOTTOM_FLOW_IMAGE_WIDTH * BOTTOM_FLOW_IMAGE_HEIGHT] __attribute__((section(".ccm")));
uint8_t buffer_reset_needed;

/* boot time in milliseconds ticks */
volatile uint32_t boot_time_ms = 0;
/* boot time in 10 microseconds ticks */
volatile uint32_t boot_time10_us = 0;

/* timer constants */
#define NTIMERS         	9
#define TIMER_CIN       	0
#define TIMER_LED       	1
#define TIMER_DELAY     	2
#define TIMER_SONAR			3
#define TIMER_SYSTEM_STATE	4
#define TIMER_RECEIVE		5
#define TIMER_PARAMS		6
#define TIMER_IMAGE			7
#define TIMER_LPOS			8
#define MS_TIMER_COUNT		100 /* steps in 10 microseconds ticks */
#define LED_TIMER_COUNT		500 /* steps in milliseconds ticks */
#define SONAR_TIMER_COUNT 	100	/* steps in milliseconds ticks */
#define SYSTEM_STATE_COUNT	1000/* steps in milliseconds ticks */
#define PARAMS_COUNT		100	/* steps in milliseconds ticks */
#define LPOS_TIMER_COUNT 	100	/* steps in milliseconds ticks */

static volatile unsigned timer[NTIMERS];
static volatile unsigned timer_ms = MS_TIMER_COUNT;


// #define EMD_COUNT (NB_SAMPLES-1)
// #define EMD_COUNT ((NB_SAMPLES_ROW)*(NB_SAMPLES_COL-1))
#define BLUR_INPUT_SIZE_X 3
const float BLUR_INPUT_KERNEL_X[] = {0.07538726f,  0.84922547f,  0.07538726f};
// #define BLUR_INPUT_SIZE_Y 1
// const float BLUR_INPUT_KERNEL_Y[] = {1.0f};
#define BLUR_INPUT_SIZE_Y 3
const float BLUR_INPUT_KERNEL_Y[] = {0.07538726f,  0.84922547f,  0.07538726f};


float emd_input[NB_EMD_INPUT * NB_SAMPLES_ROW] __attribute__((section(".ccm")));
float emd_output[NB_EMD_OUTPUT] __attribute__((section(".ccm")));
float emd_smooth[NB_EMD_OUTPUT] __attribute__((section(".ccm")));
EMD_array emd[NB_SAMPLES_ROW] __attribute__((section(".ccm")));

struct s_dir_2d {
    int16_t x[NB_SAMPLES];
    int16_t y[NB_SAMPLES];
} s_dir_2d __attribute__((section(".ccm")));

/* timer/system booleans */
bool send_system_state_now = true;
bool receive_now = true;
bool send_params_now = true;
bool send_image_now = true;
bool send_lpos_now = true;

/* local position estimate without orientation, useful for unit testing w/o FMU */
// static struct lpos_t {
// 	float x;
// 	float y;
// 	float z;
// 	float vx;
// 	float vy;
// 	float vz;
// } lpos;

/**
  * @brief  Increment boot_time_ms variable and decrement timer array.
  * @param  None
  * @retval None
  */
void timer_update_ms(void)
{
	boot_time_ms++;

	/* each timer decrements every millisecond if > 0 */
	for (unsigned i = 0; i < NTIMERS; i++)
		if (timer[i] > 0)
			timer[i]--;


	if (timer[TIMER_LED] == 0)
	{
		/* blink activitiy */
		LEDToggle(LED_ACT);
		timer[TIMER_LED] = LED_TIMER_COUNT;
	}

	if (timer[TIMER_SONAR] == 0)
	{
		sonar_trigger();
		timer[TIMER_SONAR] = SONAR_TIMER_COUNT;
	}

	if (timer[TIMER_SYSTEM_STATE] == 0)
	{
		send_system_state_now = true;
		timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;
	}

	if (timer[TIMER_RECEIVE] == 0)
	{
		receive_now = true;
		timer[TIMER_RECEIVE] = SYSTEM_STATE_COUNT;
	}

	if (timer[TIMER_PARAMS] == 0)
	{
		send_params_now = true;
		timer[TIMER_PARAMS] = PARAMS_COUNT;
	}

	if (timer[TIMER_IMAGE] == 0)
	{
		send_image_now = true;
		timer[TIMER_IMAGE] = global_data.param[PARAM_VIDEO_RATE];
	}

	if (timer[TIMER_LPOS] == 0)
	{
		send_lpos_now = true;
		timer[TIMER_LPOS] = LPOS_TIMER_COUNT;
	}
}

/**
  * @brief  Increment boot_time10_us variable and decrement millisecond timer, triggered by timer interrupt
  * @param  None
  * @retval None
  */
void timer_update(void)
{
	boot_time10_us++;

	/*  decrements every 10 microseconds*/
	timer_ms--;

	if (timer_ms == 0)
	{
		timer_update_ms();
		timer_ms = MS_TIMER_COUNT;
	}

}


uint32_t get_boot_time_ms(void)
{
	return boot_time_ms;
}

uint32_t get_boot_time_us(void)
{
	return boot_time10_us*10;// *10 to return microseconds
}

void delay(unsigned msec)
{
	timer[TIMER_DELAY] = msec;
	while (timer[TIMER_DELAY] > 0) {};
}

void buffer_reset(void) {
	buffer_reset_needed = 1;
}



/**
  * @brief  Main function.
  */
int main(void)
{
	__enable_irq();

	/* load settings and parameters */
	global_data_reset_param_defaults();
	global_data_reset();
	PROBE_INIT();
	/* init led */
	LEDInit(LED_ACT);
	LEDInit(LED_COM);
	LEDInit(LED_ERR);
	LEDOff(LED_ACT);
	LEDOff(LED_COM);
	LEDOff(LED_ERR);
        board_led_rgb(255,255,255, 1);
        board_led_rgb(  0,  0,255, 0);
        board_led_rgb(  0,  0, 0, 0);
        board_led_rgb(255,  0,  0, 1);
        board_led_rgb(255,  0,  0, 2);
        board_led_rgb(255,  0,  0, 3);
                board_led_rgb(  0,255,  0, 3);
        board_led_rgb(  0,  0,255, 4);

	/* enable FPU on Cortex-M4F core */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

	/* init clock */
	if (SysTick_Config(SystemCoreClock / 100000))/*set timer to trigger interrupt every 10 microsecond */
	{
		/* capture clock error */
		LEDOn(LED_ERR);
		while (1);
	}

	/* init usb */
	USBD_Init(	&USB_OTG_dev,
				USB_OTG_FS_CORE_ID,
				&USR_desc,
				&USBD_CDC_cb,
				&USR_cb);

	/* init mavlink */
	communication_init();

	/* enable image capturing */
	enable_image_capture();

	/* gyro config */
	gyro_config();

	/* init and clear fast image buffers */
	for (int i = 0; i < global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT]; i++)
	{
		image_buffer_8bit_1[i] = 0;
		image_buffer_8bit_2[i] = 0;
	}

	uint8_t * current_image = image_buffer_8bit_1;
	uint8_t * previous_image = image_buffer_8bit_2;

	/* usart config*/
	usart_init();

    /* i2c config*/
    i2c_init();

	/* sonar config*/
	float sonar_distance_filtered = 0.0f; // distance in meter
	float sonar_distance_raw = 0.0f; // distance in meter
	bool distance_valid = false;
	sonar_config();

	/* reset/start timers */
	timer[TIMER_SONAR] = SONAR_TIMER_COUNT;
	timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;
	timer[TIMER_RECEIVE] = SYSTEM_STATE_COUNT / 2;
	timer[TIMER_PARAMS] = PARAMS_COUNT;
	timer[TIMER_IMAGE] = global_data.param[PARAM_VIDEO_RATE];

	/* variables */
	uint32_t counter = 0;
	uint8_t qual = 0;

    // float emd_output[EMD_COUNT];
    // float emd_smooth[EMD_COUNT];
    // float emd_input[NB_SAMPLES] __attribute__((section(".ccm")));
    // float emd_output[NB_SAMPLES_COL-1] __attribute__((section(".ccm")));
    // float emd_smooth[NB_SAMPLES_COL-1] __attribute__((section(".ccm")));
    // EMD_array emd[NB_SAMPLES_ROW];

    // Array of EMD detectors
    // EMD emd[EMD_COUNT];
    // EMD_array emd;

    // float dt = 0.0018;
    // float tau = 0.02f;
    // float dt  = 0.0032;
    float dt  = 0.0061;
    // float dt  = 0.0077;
    // float tau = 0.010f;
    float tau = 0.010f;
    // float tau = 0.005f;
    float lpf_alpha = dt / (tau + dt);
    float hpf_alpha = tau / (tau + dt);
    // for (size_t i = 0; i < EMD_COUNT; i++)
    // {
    //     EMD_init(&emd[i], lpf_alpha, hpf_alpha);
    // }

    // EMD_array_init(&emd, lpf_alpha, hpf_alpha);

    for (size_t i = 0; i < NB_SAMPLES_ROW; i++)
    {
        EMD_array_init(&emd[i], lpf_alpha, hpf_alpha);
    }


    // Initialise dir2d
    for (size_t i = 0; i < NB_SAMPLES_ROW; i++)
    {
        for (size_t j = 0; j < NB_SAMPLES_COL; j++)
        {
            s_dir_2d.x[i*NB_SAMPLES_COL + j] = x_samples_2d[j];
            s_dir_2d.y[i*NB_SAMPLES_COL + j] = y_samples_2d[i];
        }
    }

	int pixel_flow_count = 0;

	static uint32_t lasttime = 0;

	uavcan_start();


	/* main loop */
	while (1)
	{
        PROBE_1(false);
        uavcan_run();
        PROBE_1(true);

		/* reset flow buffers if needed */
		if(buffer_reset_needed)
		{
			buffer_reset_needed = 0;
			for (int i = 0; i < global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT]; i++)
			{
				image_buffer_8bit_1[i] = 0;
				image_buffer_8bit_2[i] = 0;
			}
			delay(500);
			continue;
		}

		/* calibration routine */
		if(FLOAT_AS_BOOL(global_data.param[PARAM_VIDEO_ONLY]))
		{
			while(FLOAT_AS_BOOL(global_data.param[PARAM_VIDEO_ONLY]))
			{
				dcmi_restart_calibration_routine();

				/* waiting for first quarter of image */
				while(get_frame_counter() < 2){}
				dma_copy_image_buffers(&current_image, &previous_image, FULL_IMAGE_SIZE, 1);

				/* waiting for second quarter of image */
				while(get_frame_counter() < 3){}
				dma_copy_image_buffers(&current_image, &previous_image, FULL_IMAGE_SIZE, 1);

				/* waiting for all image parts */
				while(get_frame_counter() < 4){}

				send_calibration_image(&previous_image, &current_image);

				if (FLOAT_AS_BOOL(global_data.param[PARAM_SYSTEM_SEND_STATE]))
                {
					communication_system_state_send();
                }

				communication_receive_usb();
				debug_message_send_one();
				communication_parameter_send();

				LEDToggle(LED_COM);
			}

			dcmi_restart_calibration_routine();
			LEDOff(LED_COM);
		}

		uint16_t image_size   = global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT];
		uint16_t image_width  = global_data.param[PARAM_IMAGE_WIDTH];
		uint16_t image_height = global_data.param[PARAM_IMAGE_HEIGHT];

		/* new gyroscope data */
		float x_rate_sensor, y_rate_sensor, z_rate_sensor;
		int16_t gyro_temp;
		gyro_read(&x_rate_sensor, &y_rate_sensor, &z_rate_sensor,&gyro_temp);

		/* gyroscope coordinate transformation */
		float x_rate = y_rate_sensor; // change x and y rates
		float y_rate = - x_rate_sensor;
		float z_rate = z_rate_sensor; // z is correct

		/* get sonar data */
		distance_valid = sonar_read(&sonar_distance_filtered, &sonar_distance_raw);

		/* reset to zero for invalid distances */
		if (!distance_valid)
		{
			sonar_distance_filtered = 0.0f;
			sonar_distance_raw = 0.0f;
		}

		static float DT;

		/* compute optical flow */
		if (FLOAT_EQ_INT(global_data.param[PARAM_SENSOR_POSITION], BOTTOM))
		{
			// update timing
			DT = get_boot_time_us() - lasttime;
			lasttime = get_boot_time_us();

			/* copy recent image to faster ram */
			dma_copy_image_buffers(&current_image, &previous_image, image_size, 1);

            // Blur image for EMD input
			filter_2D(current_image,
						image_width,
						image_height,
						BLUR_INPUT_KERNEL_X,
						BLUR_INPUT_SIZE_X,
						BLUR_INPUT_KERNEL_Y,
						BLUR_INPUT_SIZE_Y,
						s_dir_2d.x,
						s_dir_2d.y,
						NB_SAMPLES,
						emd_input);

            // Update EMDs and sum output by columns
            // for (size_t j = 0; j < NB_SAMPLES_COL-1; j++)
            // {
            //     emd_output[j] = 0.0f;
            //     for (size_t i = 0; i < NB_SAMPLES_ROW; i++)
            //     {
            //         emd_output[j] += EMD_update(&emd[i*NB_SAMPLES_COL + j],
            //                                     emd_input[i*NB_SAMPLES_COL + j],
            //                                     emd_input[i*NB_SAMPLES_COL + j + 1]);
            //     }
            //     emd_output[j] /= NB_SAMPLES_ROW;
            // }

            // EMD_array_update(&emd[0], &emd_input[0], emd_output);

            for (size_t j = 0; j < NB_EMD_OUTPUT; j++)
            {
                emd_output[j] = 0.0f;
            }
            for (size_t i = 0; i < NB_SAMPLES_ROW; i++)
            {
                float out[NB_EMD_OUTPUT];
                EMD_array_update(&emd[i], &emd_input[i*NB_SAMPLES_COL], out);
                for (size_t j = 0; j < NB_EMD_OUTPUT; j++)
                {
                    emd_output[j] += out[j];
                }
            }
            for (size_t j = 0; j < NB_EMD_OUTPUT; j++)
            {
                emd_output[j] /= NB_SAMPLES_ROW;
            }

            // Gaussian blur with sigma=2 (4degrees)
            // #define GAUSS_WINDOW_SIZE 15
            // float gauss_window[GAUSS_WINDOW_SIZE] = {   0.00218749,  0.011109  ,  0.04393693,  0.13533528,  0.32465247,
            //                                             0.60653066,  0.8824969 ,  1.        ,  0.8824969 ,  0.60653066,
            //                                             0.32465247,  0.13533528,  0.04393693,  0.011109  ,  0.00218749 };

            // Gaussian blur with sigma=5 (10degrees)
            // #define GAUSS_WINDOW_SIZE 21
            // float gauss_window[GAUSS_WINDOW_SIZE] = {   0.13533528,  0.1978987 ,  0.2780373 ,  0.3753111 ,  0.48675226,
            //                                             0.60653066,  0.72614904,  0.83527021,  0.92311635,  0.98019867,
            //                                             1.        ,  0.98019867,  0.92311635,  0.83527021,  0.72614904,
            //                                             0.60653066,  0.48675226,  0.3753111 ,  0.2780373 ,  0.1978987 ,
            //                                             0.13533528 };

            // Gaussian kernel with sigma=10 (20degrees) with 81 samples
            // #define GAUSS_WINDOW_SIZE 41
            // float gauss_window[GAUSS_WINDOW_SIZE] = {   0.13533528,  0.16447446,  0.1978987 ,  0.23574608,  0.2780373 ,
            //                                             0.32465247,  0.3753111 ,  0.42955736,  0.48675226,  0.54607443,
            //                                             0.60653066,  0.66697681,  0.72614904,  0.78270454,  0.83527021,
            //                                             0.8824969 ,  0.92311635,  0.95599748,  0.98019867,  0.99501248,
            //                                             1.        ,  0.99501248,  0.98019867,  0.95599748,  0.92311635,
            //                                             0.8824969 ,  0.83527021,  0.78270454,  0.72614904,  0.66697681,
            //                                             0.60653066,  0.54607443,  0.48675226,  0.42955736,  0.3753111 ,
            //                                             0.32465247,  0.2780373 ,  0.23574608,  0.1978987 ,  0.16447446,
            //                                             0.13533528  };

            // Gaussian kernel with sigma=20 (20degrees) with 161 samples
            #define GAUSS_WINDOW_SIZE 81
            float gauss_window[GAUSS_WINDOW_SIZE] = {   0.13533528,  0.14938178,  0.16447446,  0.18063985,  0.1978987 ,
                                                        0.21626517,  0.23574608,  0.25634015,  0.2780373 ,  0.30081795,
                                                        0.32465247,  0.3495006 ,  0.3753111 ,  0.40202138,  0.42955736,
                                                        0.45783336,  0.48675226,  0.51620567,  0.54607443,  0.57622907,
                                                        0.60653066,  0.63683161,  0.66697681,  0.69680478,  0.72614904,
                                                        0.7548396 ,  0.78270454,  0.80957165,  0.83527021,  0.85963276,
                                                        0.8824969 ,  0.90370708,  0.92311635,  0.94058806,  0.95599748,
                                                        0.96923323,  0.98019867,  0.98881304,  0.99501248,  0.99875078,
                                                        1.        ,  0.99875078,  0.99501248,  0.98881304,  0.98019867,
                                                        0.96923323,  0.95599748,  0.94058806,  0.92311635,  0.90370708,
                                                        0.8824969 ,  0.85963276,  0.83527021,  0.80957165,  0.78270454,
                                                        0.7548396 ,  0.72614904,  0.69680478,  0.66697681,  0.63683161,
                                                        0.60653066,  0.57622907,  0.54607443,  0.51620567,  0.48675226,
                                                        0.45783336,  0.42955736,  0.40202138,  0.3753111 ,  0.3495006 ,
                                                        0.32465247,  0.30081795,  0.2780373 ,  0.25634015,  0.23574608,
                                                        0.21626517,  0.1978987 ,  0.18063985,  0.16447446,  0.14938178,
                                                        0.13533528  };

            // Gaussian kernel with sigma=50 (35degrees) with 241 samples
            // #define GAUSS_WINDOW_SIZE 201
            // float gauss_window[GAUSS_WINDOW_SIZE] = {   0.13533528,  0.14083025,  0.14648972,  0.15231569,  0.15831002,
            //                                             0.16447446,  0.17081059,  0.17731987,  0.18400359,  0.19086288,
            //                                             0.1978987 ,  0.20511182,  0.21250282,  0.22007211,  0.22781987,
            //                                             0.23574608,  0.24385049,  0.25213264,  0.26059182,  0.2692271 ,
            //                                             0.2780373 ,  0.28702097,  0.29617642,  0.30550168,  0.31499453,
            //                                             0.32465247,  0.33447271,  0.34445218,  0.35458755,  0.36487516,
            //                                             0.3753111 ,  0.38589113,  0.39661073,  0.4074651 ,  0.41844911,
            //                                             0.42955736,  0.44078414,  0.45212346,  0.46356902,  0.47511424,
            //                                             0.48675226,  0.49847592,  0.5102778 ,  0.5221502 ,  0.53408515,
            //                                             0.54607443,  0.55810956,  0.57018181,  0.58228224,  0.59440166,
            //                                             0.60653066,  0.61865965,  0.63077882,  0.6428782 ,  0.65494763,
            //                                             0.66697681,  0.67895529,  0.69087249,  0.70271772,  0.7144802 ,
            //                                             0.72614904,  0.73771331,  0.74916202,  0.76048416,  0.77166867,
            //                                             0.78270454,  0.79358073,  0.80428628,  0.81481026,  0.82514182,
            //                                             0.83527021,  0.84518478,  0.85487502,  0.86433055,  0.87354119,
            //                                             0.8824969 ,  0.89118789,  0.89960455,  0.90773754,  0.91557774,
            //                                             0.92311635,  0.93034481,  0.9372549 ,  0.9438387 ,  0.95008863,
            //                                             0.95599748,  0.96155838,  0.96676484,  0.97161077,  0.97609047,
            //                                             0.98019867,  0.98393051,  0.98728157,  0.99024786,  0.99282586,
            //                                             0.99501248,  0.99680511,  0.99820162,  0.99920032,  0.99980002,
            //                                             1.        ,  0.99980002,  0.99920032,  0.99820162,  0.99680511,
            //                                             0.99501248,  0.99282586,  0.99024786,  0.98728157,  0.98393051,
            //                                             0.98019867,  0.97609047,  0.97161077,  0.96676484,  0.96155838,
            //                                             0.95599748,  0.95008863,  0.9438387 ,  0.9372549 ,  0.93034481,
            //                                             0.92311635,  0.91557774,  0.90773754,  0.89960455,  0.89118789,
            //                                             0.8824969 ,  0.87354119,  0.86433055,  0.85487502,  0.84518478,
            //                                             0.83527021,  0.82514182,  0.81481026,  0.80428628,  0.79358073,
            //                                             0.78270454,  0.77166867,  0.76048416,  0.74916202,  0.73771331,
            //                                             0.72614904,  0.7144802 ,  0.70271772,  0.69087249,  0.67895529,
            //                                             0.66697681,  0.65494763,  0.6428782 ,  0.63077882,  0.61865965,
            //                                             0.60653066,  0.59440166,  0.58228224,  0.57018181,  0.55810956,
            //                                             0.54607443,  0.53408515,  0.5221502 ,  0.5102778 ,  0.49847592,
            //                                             0.48675226,  0.47511424,  0.46356902,  0.45212346,  0.44078414,
            //                                             0.42955736,  0.41844911,  0.4074651 ,  0.39661073,  0.38589113,
            //                                             0.3753111 ,  0.36487516,  0.35458755,  0.34445218,  0.33447271,
            //                                             0.32465247,  0.31499453,  0.30550168,  0.29617642,  0.28702097,
            //                                             0.2780373 ,  0.2692271 ,  0.26059182,  0.25213264,  0.24385049,
            //                                             0.23574608,  0.22781987,  0.22007211,  0.21250282,  0.20511182,
            //                                             0.1978987 ,  0.19086288,  0.18400359,  0.17731987,  0.17081059,
            //                                             0.16447446,  0.15831002,  0.15231569,  0.14648972,  0.14083025,
            //                                             0.13533528 };

            // Apply gaussian kernel
            filter_1D(emd_output, NB_EMD_OUTPUT, gauss_window, GAUSS_WINDOW_SIZE, emd_smooth);

            float  stats[6*SECTOR_COUNT];
			float* maxima  	= stats;
			float* minima 	= stats +   SECTOR_COUNT;
			float* max_pos  = stats + 2*SECTOR_COUNT;
			float* min_pos 	= stats + 3*SECTOR_COUNT;
			float* avg  	= stats + 4*SECTOR_COUNT;
			float* stddev  	= stats + 5*SECTOR_COUNT;

			calc_stats_1D(  NB_EMD_OUTPUT,
							SECTOR_COUNT,
							-PI/2,
							 PI/2,
							emd_smooth,
							s_dir_theta,
							maxima,
							max_pos,
							minima,
							min_pos,
							stddev,
							avg);

            // Compute psi
            float psi = 0.0f;
            float loc_max_front = 0.0f;
            float loc_max_rear = 0.0f;
            {
                float max_rear = 0.0f;
                for (size_t i = 0; i < SECTOR_COUNT/2; i++)
                {
                    if (maxima[i] > max_rear)
                    {
                        max_rear     = maxima[i];
                        loc_max_rear = max_pos[i];
                    }
                }

                float max_front = 0.0f;
                for (size_t i = SECTOR_COUNT/2; i < SECTOR_COUNT; i++)
                {
                    if (maxima[i] > max_front)
                    {
                        max_front     = maxima[i];
                        loc_max_front = max_pos[i];
                    }
                }
                psi = (loc_max_front - loc_max_rear) / 2.0f;
            }


			pixel_flow_count++;

			if (counter % (uint32_t)global_data.param[PARAM_BOTTOM_FLOW_SERIAL_THROTTLE_FACTOR] == 0)
			{
                // Send EMD information
                mavlink_msg_big_debug_vect_send(MAVLINK_COMM_0, "STATS", get_boot_time_us(), stats);

                // Send sonar data
                mavlink_msg_distance_sensor_send(MAVLINK_COMM_0, get_boot_time_ms(), 20, 500, sonar_distance_raw * 100, 0, 1, 0, 0);
			}

		    counter++;

    	    // serial mavlink  + usb mavlink output throttled
			if (counter % (uint32_t)global_data.param[PARAM_BOTTOM_FLOW_SERIAL_THROTTLE_FACTOR] == 0)
			{
				if (FLOAT_AS_BOOL(global_data.param[PARAM_USB_SEND_FLOW]))
				{
                    // Send flow on USB
                    mavlink_msg_optical_flow_send(MAVLINK_COMM_2, get_boot_time_us(), global_data.param[PARAM_SENSOR_ID],
                            emd_smooth[20], emd_smooth[30], emd_smooth[50], emd_smooth[60],
                            qual,
                            DT);
                    // // mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "DT", get_boot_time_us(), DT, 0.0, 0.0);
                    //
                    // // Send EMD maxs
                    mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "MAX02", get_boot_time_us(), max_pos[0], max_pos[1], max_pos[2]);
                    mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "MAX35", get_boot_time_us(), max_pos[3], max_pos[4], max_pos[5]);
                    mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "MAX68", get_boot_time_us(), max_pos[6], max_pos[7], max_pos[8]);
                    mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "MAX9", get_boot_time_us(), max_pos[9], 0.0f, 0.0f);
    				mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "PSI", get_boot_time_us(), loc_max_front, loc_max_rear, psi);
				}


				if(FLOAT_AS_BOOL(global_data.param[PARAM_USB_SEND_GYRO]))
				{
					mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "GYRO", get_boot_time_us(), 1000*x_rate, 1000*y_rate, 1000*z_rate);
				}
			}
		}

		// /* forward flow from other sensors */
		// if (counter % 2)
		// {
		// 	communication_receive_forward();
		// }

		/* send system state, receive commands */
		if (send_system_state_now)
		{
			/* every second */
			if (FLOAT_AS_BOOL(global_data.param[PARAM_SYSTEM_SEND_STATE]))
			{
					communication_system_state_send();
			}
			send_system_state_now = false;
		}

		/* receive commands */
		if (receive_now)
		{
			/* test every second */
			communication_receive();
			communication_receive_usb();
			receive_now = false;
		}

		/* sending debug msgs and requested parameters */
		if (send_params_now)
		{
			debug_message_send_one();
			communication_parameter_send();
			send_params_now = false;
		}

		/*  transmit raw 8-bit image */
		if (FLOAT_AS_BOOL(global_data.param[PARAM_USB_SEND_VIDEO]) && send_image_now)
		{
			/* get size of image to send */
			uint16_t image_size_send;
			uint16_t image_width_send;
			uint16_t image_height_send;

			image_size_send = image_size;
			image_width_send = global_data.param[PARAM_IMAGE_WIDTH];
			image_height_send = global_data.param[PARAM_IMAGE_HEIGHT];

			mavlink_msg_data_transmission_handshake_send(
					MAVLINK_COMM_2,
					MAVLINK_DATA_STREAM_IMG_RAW8U,
					image_size_send,
					image_width_send,
					image_height_send,
					image_size_send / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1,
					MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN,
					100);

			LEDToggle(LED_COM);
			uint16_t frame = 0;

			for (frame = 0; frame < image_size_send / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1; frame++)
			{
				mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2, frame, &((uint8_t *) current_image)[frame * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN]);
			}

			send_image_now = false;
		}
		else if (!FLOAT_AS_BOOL(global_data.param[PARAM_USB_SEND_VIDEO]))
		{
			LEDOff(LED_COM);
		}
	}
}
