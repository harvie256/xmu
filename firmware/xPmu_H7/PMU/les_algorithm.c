/*
 * les_algorithm.c
 *
 *  Created on: 19 Jul 2020
 *      Author: derry
 */

/*
 * Gain of coef = 2**27
 */


#include "main.h"
#include "les_algorithm.h"

// Constant headers with the filter coefficients
#include "les_header.h"
#include "les_dc_header.h"
#include "les_harmonics_header.h"

#define ARM_MATH_CM4
#define ARM_MATH_ROUNDING

//#include "arm_math.h" // ARM DSP math lib
#include "math.h" // standard C math library


typedef struct {
	int32_t phi; // Q31, where -180 = -1, +180 = 1
	uint32_t mag;
} les_filter_out_t;

// The buffer length is doubled so that any long processing that is done during the normal cycle has enough time
// to complete without being overwritten.
#define LES_BUFFER_LENGTH 98 // (1.5 * 32 samples/cycle + 1) * 2


void les_filter_samples(les_filter_out_t *filter_out, uint16_t est_freq, int8_t read_head, uint8_t f_estimate_mag);

const int32_t les_freq_start = 4500;
const int32_t les_harmonic_freq_start = 4950;


volatile uint8_t les_sample_buffer_head = 0;

int32_t les_angle_rotation;
volatile int32_t les_freq;
int32_t les_freq_previous;
les_filter_out_t les_output1, les_output2;


void les_init(void){
	les_freq_previous = 0;
	les_freq = 5000;
}

/*
 * LES Algorithm circular buffer
 */
int32_t les_sample_buffer[3][LES_BUFFER_LENGTH];


void les_set_sample_buffer_value(uint8_t channel, int32_t value){
	les_sample_buffer[channel][les_sample_buffer_head] = value;
}


void les_inc_buffer_head(void){
	les_sample_buffer_head++;
	if(les_sample_buffer_head >= LES_BUFFER_LENGTH){
		les_sample_buffer_head = 0;
	}
}

/*
 * les_filter_samples performs the MAC operations on the samples and provides back
 * the phi and mag estimates into the les_filter_out structure.
 *
 * This uses the global LES buffer for the samples, and the arg for the est freq.
 * The est freq is provided in Hz*100, e.g. 50Hz == 5000
 */

void les_filter_samples(les_filter_out_t *filter_out, uint16_t est_freq, int8_t read_head, uint8_t f_estimate_mag){
	int64_t real_acc = 0;
	int64_t imag_acc = 0;

	// Saturate the est_freq in case it has gone awry.
	if(est_freq > 5500)
		est_freq = 5500;
	if(est_freq < 4500)
		est_freq = 4500;

	// Perform the MAC operations
	uint16_t row = est_freq - les_freq_start;
	int8_t read_pos = read_head; // the sample buffer head indicates the next position to be written, so it will currently point at an invalid position
	for(uint8_t i=0; i<48; i++){
		read_pos -= 1;
		if(read_pos < 0) read_pos = LES_BUFFER_LENGTH - 1;

		real_acc += ((int64_t)(les_sample_buffer[0][read_pos]) * (int64_t)(les_coeff[row][0][i]));
		imag_acc += ((int64_t)(les_sample_buffer[0][read_pos]) * (int64_t)(les_coeff[row][1][i]));

	}

	// Input Q23
	// Multiplied by Q4.27 = 5.50 + q6.0 = Q11.50  However the gain is 1/16 and only 60 adds so only Q6.50

	// accumulators are Q7.54 at this point.  0.23*0.31->1.54 with 60 adds which 2^6=64, therefore 1.54+6.0->Q7.54
	// However because the operational will never equal 64 full adds, the top most bit will never be set, so effectively Q6.54

	// Calculate the angle on the full accumulator precision
	double angle = atan2((double)-imag_acc, (double)real_acc);
	filter_out->phi = angle * (683565275); // (2**31-1)/np.pi = 683565275


	// If it's not on the reporting loop, the magnitude is ignored, so there's no point calculating it.
	if(f_estimate_mag){
		// Taskes about 12us to complete the squaring and square-root

		double mag = sqrt((double)imag_acc * (double)imag_acc + (double)real_acc * (double)real_acc);
		// This will max out just under 1000VRMS
		uint32_t mag_out = ((uint64_t)mag) >> 19;

		filter_out->mag = mag_out;

	}
}

 // TODO: Include phase correction for biquad filter
void run_les(uint8_t f_reporting_cycle, les_fundamental_output_t* output){
	if(les_freq_previous == les_freq){
		// this means there is no frequency update in the last calculation, therefore there's no need to recalculate the previous window's phase,
		// we can just copy it across and use it in this iteration.
		les_output2 = les_output1;
	} else {
		// Because the frequency did change in the last estimate, we now need to recalculate the phase of the previous window.
		// Never calculate the magnitude for this filter, as only the phase is used.
		int8_t previous_read_pos = les_sample_buffer_head -1;
		if(previous_read_pos < 0) previous_read_pos = LES_BUFFER_LENGTH - 1;
		les_filter_samples(&les_output2, les_freq, previous_read_pos, 0);
	}

	// Calculate the phase estimate for the current sample, and the magnitude if it's a reporting cycle
	les_filter_samples(&les_output1, les_freq, les_sample_buffer_head, f_reporting_cycle);

	// Calculate the phase rotation.
	les_angle_rotation = -(les_output1.phi - les_output2.phi);

	// Store the frequency estimate from the previous window, which was used for phase estimation of this window, for comparison in the next window
	les_freq_previous = les_freq;

	/*
	 * Angular rotation to freq_x100
	 *
	 * rotation in -1 to 1 space, equivalent to -pi to pi
	 *
	 * 2000 (from sps) * 0.5 (from one rotation == 2) * 100 (as we want it in a 100 format)
	 *
	 * rotation is in Q31 format, output needs to be Q31.0 with a 100 multiplier
	 *
	 */
	les_freq = (((int64_t)les_angle_rotation * 160000) + 0x80000000) >> 32;
	output->frequency = les_freq;
	output->fundamental_mag[0] = les_output1.mag;
	output->phase[0] = les_output1.phi;
}


/*
 * Calculates the harmonics content.  Returns 1 if frequency is outside of the range of the header.
 *
 * Currently only running on channel 0
 */
int8_t les_calculate_harmonics(les_harmonics_output_t* harmonics_output){
	int64_t real_acc[8] = { 0 };
	int64_t imag_acc[8] = { 0 };
	int32_t freq = les_freq; // This will change in the interrupt routines, so we need to capture it at the start.
	int32_t read_head = les_sample_buffer_head;

	// Check if the current fundamental frequency is within the bounds that the harmonics header contains
	if(freq < 4950 ||
			freq > 5050){
		return 1;
	}

	uint16_t row = freq - les_harmonic_freq_start;
	int8_t read_pos = read_head; // the sample buffer head indicates the next position to be written, so it will currently point at an invalid position
	for(uint8_t i=0; i<48; i++){
		read_pos -= 1;
		if(read_pos < 0) read_pos = LES_BUFFER_LENGTH - 1;

		for(uint8_t j=0; j<8; j++){
			real_acc[j] += ((int64_t)(les_sample_buffer[0][read_pos]) * (int64_t)(les_harmonics_coeff[row][2*j + 0][i]));
			imag_acc[j] += ((int64_t)(les_sample_buffer[0][read_pos]) * (int64_t)(les_harmonics_coeff[row][2*j + 1][i]));
		}

	}

	for(uint8_t i=0; i<8; i++){
		double mag = sqrt((double)imag_acc[i] * (double)imag_acc[i] + (double)real_acc[i] * (double)real_acc[i]);
		// This will max out just under 1000VRMS
		uint32_t mag_out = ((uint64_t)mag) >> 19;

		harmonics_output->harmonic_mag[i] = mag_out;

	}

	return 0;
}



