#include "SimpleDCMotor.h"

class CustomDriver : public DCDriver2PWM {
    public:
        using DCDriver2PWM::DCDriver2PWM;

        virtual void setPwm(float U) override {
            if (abs(U) < 0.1) {
                _writeDutyCycle2PWM(0, 0, params);
            } else {
                DCDriver2PWM::setPwm(U);
            }
        }
};
