/*!
 *  \file
 *  \brief  Low pass IIR filter source file
 *  \author Alexey Vasilenko a.vasilenko@docet.ai
*/

#include "low_pass_simple.h"

float sampling_freq;
float time_const;
float val_acc;
float out;

void filter_init(float sampl_freq, float t_const) {
	sampling_freq = sampl_freq;
	time_const = (float)t_const/1000.0;
	val_acc = 0.0;
	out = 0.0;
}

/*!
 *  \brief      Used for data filtration
 *  \param[in]  x New input value
 *  \return     Filtered value
 */
float filter_process(float x) {
	val_acc = val_acc + x - out;
	out = val_acc/(sampling_freq * time_const);
	return out;
}

