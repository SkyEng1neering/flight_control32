# PID library
Universal PID library

## Authors
Vasilenko Alexey a.vasilenko@docet.ai
Petr Antipov p.antipov@docet.ai

## Getting started
Just create PID class object, set sample period, output limits and P, I and D constants. Then you can set target value and call **calculate()** method with sample period to get the value for regulatory influence.


## Usage example
```cpp
#include "pidlib.h"

PID<float> pid;
void foo() {
    pid.setConstants(1.5, 7.5, 0.3);
    pid.setSamplePeriod(10.0);
    pid.setOutputLimits(0.0, 150.0);
    pid.setTarget(65.0);

    while (1) {
        float val = calculate(getSensorValue());
        setOutputValue(val);
        delay_ms(10);
    }
}

```
Example of using this library you can find in [**cic_app**](https://docet.gitlab.yandexcloud.net/hi2m-karts/embedded_software/cic/cic_app) project.
