/*
 * les_algorithm.h
 *
 *  Created on: 19 Jul 2020
 *      Author: derry
 */

#ifndef INC_LES_ALGORITHM_H_
#define INC_LES_ALGORITHM_H_


typedef struct {
	int32_t frequency;
	int32_t phase[3];
	uint32_t fundamental_mag[3];
} les_fundamental_output_t;

typedef struct {
	uint32_t dc_offset;
	uint32_t harmonic_mag[8];
}les_harmonics_output_t;

void les_set_sample_buffer_value(uint8_t channel, int32_t value);
void les_inc_buffer_head(void);
void les_init(void);
void run_les(uint8_t f_reporting_cycle, les_fundamental_output_t* output);
int8_t les_calculate_harmonics(les_harmonics_output_t* harmonics_output);

#endif /* INC_LES_ALGORITHM_H_ */
