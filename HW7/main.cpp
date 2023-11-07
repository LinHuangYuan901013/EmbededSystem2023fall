#include "mbed.h"


#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"
#include <iostream>
#include <fstream>
#include "FATFileSystem.h"
using namespace std;

// save data and filter data and plot 2 pictures
/* ----------------------------------------------------------------------
 * Copyright (C) 2010-2012 ARM Limited. All rights reserved.
 *
* $Date:         17. January 2013
* $Revision:     V1.4.0
*
* Project:       CMSIS DSP Library
 * Title:        arm_fir_example_f32.c
 *
 * Description:  Example code demonstrating how an FIR filter can be used
 *               as a low pass filter.
 *
 * Target Processor: Cortex-M4/Cortex-M3
 *
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------- */

/**
 * @ingroup groupExamples
 */

/**
 * @defgroup FIRLPF FIR Lowpass Filter Example
 *
 * \par Description:
 * \par
 * Removes high frequency signal components from the input using an FIR lowpass filter.
 * The example demonstrates how to configure an FIR filter and then pass data through
 * it in a block-by-block fashion.
 * \image html FIRLPF_signalflow.gif
 *
 * \par Algorithm:
 * \par
 * The input signal is a sum of two sine waves:  1 kHz and 15 kHz.
 * This is processed by an FIR lowpass filter with cutoff frequency 6 kHz.
 * The lowpass filter eliminates the 15 kHz signal leaving only the 1 kHz sine wave at the output.
 * \par
 * The lowpass filter was designed using MATLAB with a sample rate of 48 kHz and
 * a length of 29 points.
 * The MATLAB code to generate the filter coefficients is shown below:
 * <pre>
 *     h = fir1(28, 6/24);
 * </pre>
 * The first argument is the "order" of the filter and is always one less than the desired length.
 * The second argument is the normalized cutoff frequency.  This is in the range 0 (DC) to 1.0 (Nyquist).
 * A 6 kHz cutoff with a Nyquist frequency of 24 kHz lies at a normalized frequency of 6/24 = 0.25.
 * The CMSIS FIR filter function requires the coefficients to be in time reversed order.
 * <pre>
 *     fliplr(h)
 * </pre>
 * The resulting filter coefficients and are shown below.
 * Note that the filter is symmetric (a property of linear phase FIR filters)
 * and the point of symmetry is sample 14.  Thus the filter will have a delay of
 * 14 samples for all frequencies.
 * \par
 * \image html FIRLPF_coeffs.gif
 * \par
 * The frequency response of the filter is shown next.
 * The passband gain of the filter is 1.0 and it reaches 0.5 at the cutoff frequency 6 kHz.
 * \par
 * \image html FIRLPF_response.gif
 * \par
 * The input signal is shown below.
 * The left hand side shows the signal in the time domain while the right hand side is a frequency domain representation.
 * The two sine wave components can be clearly seen.
 * \par
 * \image html FIRLPF_input.gif
 * \par
 * The output of the filter is shown below.  The 15 kHz component has been eliminated.
 * \par
 * \image html FIRLPF_output.gif
 *
 * \par Variables Description:
 * \par
 * \li \c testInput_f32_1kHz_15kHz points to the input data
 * \li \c refOutput points to the reference output data
 * \li \c testOutput points to the test output data
 * \li \c firStateF32 points to state buffer
 * \li \c firCoeffs32 points to coefficient buffer
 * \li \c blockSize number of samples processed at a time
 * \li \c numBlocks number of frames
 *
 * \par CMSIS DSP Software Library Functions Used:
 * \par
 * - arm_fir_init_f32()
 * - arm_fir_f32()
 *
 * <b> Refer  </b>
 * \link arm_fir_example_f32.c \endlink
 *
 */


/** \example arm_fir_example_f32.c
 */

/* ----------------------------------------------------------------------
** Include Files
** ------------------------------------------------------------------- */

#include "arm_math.h"
#include "math_helper.h"
#include <cstdio>
#define SEMIHOSTING 1
#if defined(SEMIHOSTING)
#include <stdio.h>
#endif


/* ----------------------------------------------------------------------
** Macro Defines
** ------------------------------------------------------------------- */

// #define TEST_LENGTH_SAMPLES  320
#define TEST_LENGTH_SAMPLES  960
/*

This SNR is a bit small. Need to understand why
this example is not giving better SNR ...

*/
#define SNR_THRESHOLD_F32    75.0f
#define BLOCK_SIZE            32

#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
/* Must be a multiple of 16 */
#define NUM_TAPS_ARRAY_SIZE              32
#else
#define NUM_TAPS_ARRAY_SIZE              29
#endif

#define NUM_TAPS              29

/* -------------------------------------------------------------------
 * The input signal and reference output (computed with MATLAB)
 * are defined externally in arm_fir_lpf_data.c.
 * ------------------------------------------------------------------- */
// testInput_f32_1kHz_15kHz contains the input signal
extern float32_t testInput_f32_1kHz_15kHz[TEST_LENGTH_SAMPLES];
float32_t sensor_InputValues[TEST_LENGTH_SAMPLES];
// refOutput contains the reference output.
extern float32_t refOutput[TEST_LENGTH_SAMPLES];

/* -------------------------------------------------------------------
 * Declare Test output buffer
 * ------------------------------------------------------------------- */
static float32_t testOutput[TEST_LENGTH_SAMPLES];
float32_t sensor_FilteredValues[TEST_LENGTH_SAMPLES];



// Define the FIR filter coefficients and state buffer 
// #define NUM_TAPS 5  // Define the number of filter taps
// float32_t firCoeffs32[NUM_TAPS] = {0.2, 0.2, 0.2, 0.2, 0.2};  // FIR filter coefficients
// float32_t firStateF32[NUM_TAPS + 2];  // State buffer



/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
static float32_t firStateF32[2 * BLOCK_SIZE + NUM_TAPS - 1];
#else
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
#endif 

#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
static float32_t pfirStateF32[2 * BLOCK_SIZE + NUM_TAPS - 1];
#else
static float32_t pfirStateF32[BLOCK_SIZE + NUM_TAPS - 1];
#endif 

int getpfirStateF32Length = sizeof(pfirStateF32) / sizeof(float32_t);
/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using fir1() MATLAB function.
** fir1(28, 6/24)
** ------------------------------------------------------------------- */
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f, 0.0f,0.0f,0.0f
};
#else
const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};
#endif




/* ------------------------------------------------------------------
 * Global variables for FIR LPF Example
 * ------------------------------------------------------------------- */
uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;

float32_t  snr;

int16_t pDataXYZ[3] = {0};
float pGyroDataXYZ[3] = {0};

// Create variables to store sensor data and filtered data for the accelerometer and gyroscope:
// int16_t accelDataXYZ[3] = {0};  // Raw accelerometer data (X, Y, Z)
// float32_t filteredAccelDataXYZ[3] = {0};  // Filtered accelerometer data (X, Y, Z)

// float32_t gyroDataXYZ[3] = {0};  // Raw gyroscope data (X, Y, Z)
// float32_t filteredGyroDataXYZ[3] = {0};  // Filtered gyroscope data (X, Y, Z)


// DigitalOut led(LED1);

// // Define FIR filter coefficients (you need to calculate these)
// const float32_t firCoeffs[NUM_TAPS] = {0.1, 0.2, 0.3, 0.2, 0.1};
// const int numTaps = 5; // Number of filter taps

// int main()
// {
//     float sensor_value = 0;
//     int16_t pDataXYZ[3] = {0};
//     float pGyroDataXYZ[3] = {0};

//     // Create arrays for previous accelerometer data and filtered data
//     float prevAccelData[3] = {0};
//     float filteredAccelData[3] = {0};

//     // Create an instance of the FIR filter
//     arm_fir_instance_f32 firFilter;
//     arm_fir_init_f32(&firFilter, numTaps, (float32_t *)firCoeffs, prevAccelData, 3);

//     printf("Start sensor init\n");

//     // Initialize sensors (already done in your code)

//     while (1) {
//         printf("\nNew loop, LED1 should blink during sensor read\n");

//         led = 1;

//         // Read accelerometer and gyroscope data
//         BSP_ACCELERO_AccGetXYZ(pDataXYZ);
//         BSP_GYRO_GetXYZ(pGyroDataXYZ);

//         // Apply FIR filter to accelerometer data
//         arm_fir_f32(&firFilter, (float32_t *)pDataXYZ, filteredAccelData, 3);

//         // Display filtered accelerometer data
//         printf("Filtered ACCELERO_X = %.2f\n", filteredAccelData[0]);
//         printf("Filtered ACCELERO_Y = %.2f\n", filteredAccelData[1]);
//         printf("Filtered ACCELERO_Z = %.2f\n", filteredAccelData[2]);

//         led = 0;

//         ThisThread::sleep_for(1000);
//     }
// }


/* ----------------------------------------------------------------------
 * FIR LPF Example
 * ------------------------------------------------------------------- */
DigitalOut led(LED1);



int32_t main(void)
{
  
  
  uint32_t i;
  arm_fir_instance_f32 S;
  arm_fir_instance_f32 accelFIR, gyroFIR;
  arm_status status;
  float32_t  *inputF32, *outputF32;
  float32_t  *pinputF32, *poutputF32;

  /* Initialize input and output buffer pointers */
  inputF32 = &testInput_f32_1kHz_15kHz[0];
  outputF32 = &testOutput[0];
  pinputF32 = &sensor_InputValues[0];
  poutputF32 = &sensor_FilteredValues[0];

    // int16_t pDataXYZ[3] = {0};
    // float pGyroDataXYZ[3] = {0};
    // float filteredAccData[3] = {0};
    // float filteredGyroData[3] = {0};
    // arm_fir_instance_f32 accelFIR, gyroFIR;
    // float32_t accelState[BLOCK_SIZE + NUM_TAPS - 1];
    // float32_t gyroState[BLOCK_SIZE + NUM_TAPS - 1];

    BSP_ACCELERO_Init();
    BSP_GYRO_Init();

    // read sensor value and store in array
    for(i=0; i < TEST_LENGTH_SAMPLES; i = i+3 )
    {   
        led = 0;
        // BSP_ACCELERO_AccGetXYZ(pDataXYZ);
        BSP_GYRO_GetXYZ(pGyroDataXYZ);
        sensor_InputValues[i] = pGyroDataXYZ[0];
        sensor_InputValues[i+1] = pGyroDataXYZ[1];
        sensor_InputValues[i+2] = pGyroDataXYZ[2];
        printf("\n%.2f, ", pGyroDataXYZ[0]);
        printf("%.2f, ", pGyroDataXYZ[1]);
        printf("%.2f, ", pGyroDataXYZ[2]);
        led = 1;
        ThisThread::sleep_for(200);
    }
    // i=0;
    // printf("getArrayLength=%d\n",sizeof(firStateF32));
    // arm_fir_init_f32(&accelFIR, NUM_TAPS, (float32_t *)&firCoeffs32[0], accelState, BLOCK_SIZE);
    // arm_fir_init_f32(&gyroFIR, NUM_TAPS, (float32_t *)&firCoeffs32[0], gyroState, BLOCK_SIZE);

    // while (1) {
        // BSP_ACCELERO_AccGetXYZ(pDataXYZ);
        // BSP_GYRO_GetXYZ(pGyroDataXYZ);

        // arm_fir_f32(&accelFIR, (float32_t *)pDataXYZ, filteredAccData, BLOCK_SIZE);
        // arm_fir_f32(&gyroFIR, pGyroDataXYZ, filteredGyroData, BLOCK_SIZE);

        // Print the original sensor data
        // printf("Original Accel Data: X=%d, Y=%d, Z=%d\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
        // printf("Original Gyro Data: X=%.2f, Y=%.2f, Z=%.2f\n", pGyroDataXYZ[0], pGyroDataXYZ[1], pGyroDataXYZ[2]);

        // Print the filtered sensor data
        // printf("Filtered Accel Data: X=%.2f, Y=%.2f, Z=%.2f\n", filteredAccData[0], filteredAccData[1], filteredAccData[2]);
        // printf("Filtered Gyro Data: X=%.2f, Y=%.2f, Z=%.2f\n", filteredGyroData[0], filteredGyroData[1], filteredGyroData[2]);

        // ThisThread::sleep_for(1s);
    // }

  /* Call FIR init function to initialize the instance structure. */
  arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);
    // arm_fir_init_f32(&accelFIR, NUM_TAPS, (float32_t *)&firCoeffs32[0], accelState, BLOCK_SIZE);
    arm_fir_init_f32(&gyroFIR, NUM_TAPS, (float32_t *)&firCoeffs32[0], &pfirStateF32[0], BLOCK_SIZE);

  
  // Initialize the FIR filter for both the accelerometer and gyroscope data:  
//   arm_fir_instance_f32 accelFIR, gyroFIR;
//   arm_fir_init_f32(&accelFIR, NUM_TAPS, firCoeffs32, firStateF32, 1);  // Initialize accelerometer FIR
//   arm_fir_init_f32(&gyroFIR, NUM_TAPS, firCoeffs32, firStateF32, 1);  // Initialize gyroscope FIR


  /* ----------------------------------------------------------------------
  ** Call the FIR process function for every blockSize samples
  ** ------------------------------------------------------------------- */

  for(i=0; i < numBlocks; i++)
  {
    arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
    arm_fir_f32(&gyroFIR, pinputF32 + (i * blockSize), poutputF32 + (i * blockSize), blockSize);
  }

  printf("\n===================filter finish=======================\n");

    for (int i = 0; i < getpfirStateF32Length; ++i) {
        cout << pfirStateF32[i] << ", ";
    }





  /* ----------------------------------------------------------------------
  ** Compare the generated output against the reference output computed
  ** in MATLAB.
  ** ------------------------------------------------------------------- */
  //  calculates the Signal-to-Noise Ratio (SNR) using the arm_snr_f32 function. 
  //  It compares the filtered output (testOutput) to the reference output (refOutput).
  snr = arm_snr_f32(&refOutput[0], &testOutput[0], TEST_LENGTH_SAMPLES);

  status = (snr < SNR_THRESHOLD_F32) ? ARM_MATH_TEST_FAILURE : ARM_MATH_SUCCESS;
  
//   if (status != ARM_MATH_SUCCESS)
//   {
//     #if defined (SEMIHOSTING)
//         printf("FAILURE\n");
//     #else
//         while (1);                             /* main function does not return */
//     #endif
//   }
//   else
//   {
//     #if defined (SEMIHOSTING)
//         printf("SUCCESS\n");
//     #else
//         while (1);                             /* main function does not return */
//     #endif
//   }
}

/** \endlink */