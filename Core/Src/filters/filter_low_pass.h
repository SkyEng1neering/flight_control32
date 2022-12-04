/*
 * FilterLowPass.h
 *
 *  Created on: Dec 5, 2022
 *      Author: Docet
 */

#ifndef SRC_FILTERS_FILTERLOWPASS_H_
#define SRC_FILTERS_FILTERLOWPASS_H_

class FilterLowPass {
private:
    float sampling_freq;
    float time_const;
    float val_acc;
    float out;

public:
    FilterLowPass(float sampl_freq, float t_const) {
        init(sampl_freq, t_const);
    }
    virtual ~FilterLowPass() {}

    void init(float sampl_freq, float t_const) {
        sampling_freq = sampl_freq;
        time_const = (float)t_const / 1000.0;
        val_acc = 0.0;
        out = 0.0;
    }

    float process(float x) {
        val_acc = val_acc + x - out;
        out = val_acc / (sampling_freq * time_const);
        return out;
    }
};

#endif /* SRC_FILTERS_FILTERLOWPASS_H_ */
