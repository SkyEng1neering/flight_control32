/*!
 *  \file
 *  \brief  Low pass IIR filter header file
 *  \author Alexey Vasilenko a.vasilenko@docet.ai
*/

#ifndef PLATFORM_INDEPENDENT_FILTER_LOW_PASS_2_LOW_PASS_FIR_H_
#define PLATFORM_INDEPENDENT_FILTER_LOW_PASS_2_LOW_PASS_FIR_H_

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  \brief      Used for data filtration
 */
float filter_process(float x);
void filter_init(float sampl_freq, float t_const);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_INDEPENDENT_FILTER_LOW_PASS_2_LOW_PASS_FIR_H_ */
