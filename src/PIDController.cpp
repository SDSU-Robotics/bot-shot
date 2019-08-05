#include "PIDController.h"

PIDController::PIDController(float kP, float kI, float kD, float ILimit, float maxOut)
{
	_kP = kP;
	_kI = kI;
	_kD = kD;
	_ILimit = ILimit;
	_maxOut = maxOut;
}

float PIDController::calcOutput(float feedback)
{
	_error = _sp - feedback;

	_integral = _integral + _error;

	if (_integral > _ILimit)
		_integral = _ILimit;

	if (_integral < -_ILimit)
		_integral = -_ILimit;

	_derivative = _error - _lastError;

	float output = _kP * _error + _kI * _integral + _kD * _derivative;

	if (output > _maxOut)
		output = _maxOut;

	if (output < -1 * _maxOut)
		output = -1 * _maxOut;

	_lastError = _error;

	return output;
}