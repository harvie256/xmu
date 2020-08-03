/*
 * decimation_filter.c
 *
 *  Created on: 14 Jul 2020
 *      Author: derry
 */

/*
 *
 * Input in Q23 format
 *
 * Total gain of filter is 2
 *
 *
 */


#include "main.h"

// Each of the voltage channels needs it's own set of delay elements
#define no_channels 3

const int64_t sos_coef[2][6] = {
		{ 160028528,  320057057,  160028528,  536870912, -892284803,  372894784},
		{ 536870912, 1073741824,  536870912,  536870912, -979949285,  462276714}};


// This is in Q31 with a factor of 1/2, so the output should be multiplied and left shifted 30 places.
const int64_t decimation_filter_harmonic_correction[9] = {
		1073741855, 1073749820, 1073947158, 1075796858, 1085977254,
		       1125632576, 1243658686, 1518500249, 2032594022
};


const int32_t q = 29;

int64_t delay_f1_in[no_channels][2] = {0};
int64_t delay_f2_in[no_channels][2] = {0};
int64_t delay_f1_out[no_channels][2] = {0};
int64_t delay_f2_out[no_channels][2] = {0};


int32_t deci_correct_harmonic_mag(int32_t magnitude, uint8_t harmonic_no){
	return ((int64_t)magnitude * decimation_filter_harmonic_correction[harmonic_no])>>30;
}

int32_t filter_sample(int32_t sample, uint8_t channel_index){

	volatile int64_t acc1 = 0, acc2 = 0;

	// First SOS section

	// Perform the MACs
	acc1 = sos_coef[0][0] * (int64_t)sample
			+ sos_coef[0][1] * delay_f1_in[channel_index][0]
			+ sos_coef[0][2] * delay_f1_in[channel_index][1]
			- sos_coef[0][4] * delay_f1_out[channel_index][0]
			- sos_coef[0][5] * delay_f1_out[channel_index][1];

	// Shift the accumulator back by the scale factor with rounding
	acc1 += 1<<(q-1);
	acc1 >>= q;

	// Update the delay line
    delay_f1_in[channel_index][1] = delay_f1_in[channel_index][0];
    delay_f1_in[channel_index][0] = sample;

    delay_f1_out[channel_index][1] = delay_f1_out[channel_index][0];
    delay_f1_out[channel_index][0] = acc1;



	// Second SOS section
    acc1 += 1<<4;
    acc1 >>= 5; // too much gain in the first filter to directly feed the second without 64b overflow


	// Perform the MACs
	acc2 = sos_coef[1][0] * acc1
			+ sos_coef[1][1] * delay_f2_in[channel_index][0]
			+ sos_coef[1][2] * delay_f2_in[channel_index][1]
			- sos_coef[1][4] * delay_f2_out[channel_index][0]
			- sos_coef[1][5] * delay_f2_out[channel_index][1];

	// Shift the accumulator back by the scale factor
	acc2 += 1<<(q-1);
	acc2 >>= q;

	// Update the delay line
    delay_f2_in[channel_index][1] = delay_f2_in[channel_index][0];
    delay_f2_in[channel_index][0] = acc1;

    delay_f2_out[channel_index][1] = delay_f2_out[channel_index][0];
    delay_f2_out[channel_index][0] = acc2;

    return acc2 >> 6;
}


