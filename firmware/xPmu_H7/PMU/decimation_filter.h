/*
 * decimation_filter.h
 *
 *  Created on: 14 Jul 2020
 *      Author: derry
 */

#ifndef INC_DECIMATION_FILTER_H_
#define INC_DECIMATION_FILTER_H_

int32_t filter_sample(int32_t sample, uint8_t channel_index);
int32_t deci_correct_harmonic_mag(int32_t magnitude, uint8_t harmonic_no);

#endif /* INC_DECIMATION_FILTER_H_ */
