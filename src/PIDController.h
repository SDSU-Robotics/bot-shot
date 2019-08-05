#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController
{
private:
    float _kP;
	float _kI;
	float _kD;
    float _ILimit;
	float _maxOut;

	float _error = 0.0;
    float _integral = 0.0;
    float _derivative = 0.0;
    float _lastError = 0.0;
    float _lastAngle = 0.0;

    float _sp = 0.0;

public:
    PIDController(float kP = 0.0, float kI = 0.0, float kD = 0.0, float ILimit = 1000, float maxOut = 1.0);

    void setKP(float kP) { _kP = kP; }
    void setKI(float kI) { _kI = kI; }
    void setKD(float kD) { _kD = kD; }
    void setILimit(float ILimit) { _ILimit = ILimit; }
    void setMaxOut(float maxOut) { _maxOut = maxOut; }
    void setSetpoint(float sp) { _sp = sp; }
    float getSetpoint() { return _sp; }
    
    float calcOutput(float feedback);
};

#endif