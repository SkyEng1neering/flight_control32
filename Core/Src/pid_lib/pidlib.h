/*!
 *  \file
 *  \brief Header file with PID template class
 *  \author Alexey Vasilenko a.vasilenko@docet.ai
 *          Petr Antipov p.antipov@docet.ai
*/

#ifndef SRC_PID_PIDLIB_H_
#define SRC_PID_PIDLIB_H_

#include <stdint.h>
#include <cmath>

/*!
 *  \brief   PID template class
 */
template <typename T>
class PID {
private:
    bool resetflag = 0;
    float kp;
    float ki;
    float kd;
    double integration;
    float period;
    T last_error;
    T min_output;
    T max_output;
    T target;

public:
    PID(float koef_p, float koef_i, float koef_d, T min_out, T max_out, float sample_period) {
        kp = koef_p;
        ki = koef_i;
        kd = koef_d;
        min_output = min_out;
        max_output = max_out;
        period = sample_period;
    }

    PID() {}

    /*!
     *  \brief     Set P, I and D constants
     *  \param[in] p P constant
     *  \param[in] i I constant
     *  \param[in] d D constant
     */
    void setConstants(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    /*!
     *  \brief     Set output limits
     *  \param[in] min Minimum limit
     *  \param[in] max Maximum limit
     */
    void setOutputLimits(T min, T max) {
        min_output = min;
        max_output = max;
    }

    /*!
     *  \brief     Set target value
     *  \param[in] tgt Target value
     */
    void setTarget(T tgt) {
        target = tgt;
    }

    /*!
     *  \brief     Reset integral component
     */
    void resetIntegrative() {
        integration = 0.0;
    }

    /*!
     *  \brief     Set sample period
     *  \param[in] t Sample period
     */
    void setSamplePeriod(float t) {
        period = t;
    }

    /*!
     *  \brief     Calculate new output value
     *  \param[in] input Sensor input
     *  \return    calculated value
     */
    T calculate(T input) {
        return calculate(input, target);
    }

    /*!
     *  \brief     Calculate new output value
     *  \param[in] input Sensor input
     *  \param[in] tgt Target value
     *  \return    calculated value
     */
    T calculate(T input, T tgt) {
        T error = tgt - input;
        /*if((error >= 0.0 &&  last_error < 0.0) || (error <= 0.0 &&  last_error > 0.0)){
            resetIntegrative(); //check anti-windup
        }*/
        integration += (double)error * period;
        integration = (integration * ki > min_output / 2  ? (integration * ki < max_output / 2 ? integration : max_output / ki / 2) : min_output / ki / 2);

        double derivation = ((double)(error - last_error)) / period;
        /*if(resetflag){
            resetDerivative();
            resetflag = false;  //check deriv reset
        }*/

        T Poutput = (T)(kp*(float)error);
        T Ioutput = (T)(ki*integration);
        T Doutput = (T)(kd*derivation);
        //Ioutput = (Ioutput > min_output / 4 ? (Ioutput < max_output / 4 ? Ioutput : max_output / 4) : min_output / 4);
        T output = Poutput + Ioutput + Doutput;
        output = (output > min_output ? (output < max_output ? output : max_output) : min_output);

        last_error = error;
        //resetflag = false;
        return output;
    }
};

#endif /* SRC_PID_PIDLIB_H_ */
