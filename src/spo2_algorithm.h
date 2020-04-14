/** \file algorithm.h ******************************************************
*
* Project: MAXREFDES117#
* Filename: algorithm.h
* Description: This module is the heart rate/SpO2 calculation algorithm header file
*
* Revision History:
*\n 1-18-2016 Rev 01.00 SK Initial release.
*\n
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
*\n char              ch_pmod_value
*\n char (array)      s_pmod_s_string[16]
*\n float             f_pmod_value
*\n int32_t           n_pmod_value
*\n int32_t (array)   an_pmod_value[16]
*\n int16_t           w_pmod_value
*\n int16_t (array)   aw_pmod_value[16]
*\n uint16_t          uw_pmod_value
*\n uint16_t (array)  auw_pmod_value[16]
*\n uint8_t           uch_pmod_value
*\n uint8_t (array)   auch_pmod_buffer[16]
*\n uint32_t          un_pmod_value
*\n int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
#ifndef SPO2_ALGORITHM_H_
#define SPO2_ALGORITHM_H_

#include <Arduino.h>

#define FreqS 25    //sampling frequency
#define BUFFER_SIZE (FreqS * 4) 
#define MA4_SIZE 4 // DONOT CHANGE
//#define min(x,y) ((x) < (y) ? (x) : (y)) //Defined in Arduino.h

//uch_spo2_table is approximated as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
              99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
              100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
              97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
              90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 
              80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
              66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
              49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
              28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
              3, 2, 1 } ;
static  int32_t an_x[ BUFFER_SIZE]; //ir
static  int32_t an_y[ BUFFER_SIZE]; //red


#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated.  Samples become 16-bit data.
void maxim_heart_rate_and_oxygen_saturation(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);
#else
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);
#endif

void maxim_find_peaks(int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);
void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height);
void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
void maxim_sort_ascend(int32_t  *pn_x, int32_t n_size);
void maxim_sort_indices_descend(int32_t  *pn_x, int32_t *pn_indx, int32_t n_size);

/*
 * Settable parameters 
 * Leave these alone if your circuit and hardware setup match the defaults 
 * described in this code's Instructable. Typically, different sampling rate
 * and/or sample length would require these paramteres to be adjusted.
 */
#define ST 4      // Sampling time in s. WARNING: if you change ST, then you MUST recalcuate the sum_X2 parameter below!
#define FS 25     // Sampling frequency in Hz. WARNING: if you change FS, then you MUST recalcuate the sum_X2 parameter below!
// Sum of squares of ST*FS numbers from -mean_X (see below) to +mean_X incremented be one. For example, given ST=4 and FS=25,
// the sum consists of 100 terms: (-49.5)^2 + (-48.5)^2 + (-47.5)^2 + ... + (47.5)^2 + (48.5)^2 + (49.5)^2
// The sum is symmetrc, so you can evaluate it by multiplying its positive half by 2. It is precalcuated here for enhanced 
// performance.
const float sum_X2 = 83325; // WARNING: you MUST recalculate this sum if you changed either ST or FS above!
#define MAX_HR 125  // Maximal heart rate. To eliminate erroneous signals, calculated HR should never be greater than this number.
#define MIN_HR 40   // Minimal heart rate. To eliminate erroneous signals, calculated HR should never be lower than this number.
// Typical heart rate. Set it to the upper value of the expected heart rate range in a given application. Obviously, it must be 
// in between MIN_HR and MAX_HR. For example, if HR in an overnight measurement varies between 46 and 65, but 90% of the time 
// stays between 50 and 60, then set it to 60.
// WARNING: This is a CRUCIAL parameter! Proper HR evaluation depends on it.
#define TYPICAL_HR 60
// Minimal ratio of two autocorrelation sequence elements: one at a considered lag to the one at lag 0.
// Good quality signals must have such ratio greater than this minimum.
const float min_autocorrelation_ratio = 0.5;
// Pearson correlation between red and IR signals.
// Good quality signals must have their correlation coefficient greater than this minimum.
const float min_pearson_correlation = 0.8;

/*
 * Derived parameters 
 * Do not touch these! 
 * 
 */
const int32_t FS60 = FS*60;  // Conversion factor for heart rate from bps to bpm
const int32_t LOWEST_PERIOD = FS60/MAX_HR; // Minimal distance between peaks
const int32_t HIGHEST_PERIOD = FS60/MIN_HR; // Maximal distance between peaks
const int32_t INIT_INTERVAL = FS60/TYPICAL_HR; // Seed value for heart rate determination routine
const float mean_X = (float)(BUFFER_SIZE-1)/2.0; // Mean value of the set of integers from 0 to BUFFER_SIZE-1. For ST=4 and FS=25 it's equal to 49.5.

void rf_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, float *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, 
                                        int8_t *pch_hr_valid, float *ratio, float *correl);
float rf_linear_regression_beta(float *pn_x, float xmean, float sum_x2);
float rf_autocorrelation(float *pn_x, int32_t n_size, int32_t n_lag);
float rf_rms(float *pn_x, int32_t n_size, float *sumsq);
float rf_Pcorrelation(float *pn_x, float *pn_y, int32_t n_size);
void rf_signal_periodicity(float *pn_x, int32_t n_size, int32_t *p_last_periodicity, int32_t n_min_distance, int32_t n_max_distance, float min_aut_ratio, float aut_lag0, float *ratio);

#endif /* ALGORITHM_H_ */

