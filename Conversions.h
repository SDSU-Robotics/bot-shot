#ifndef CONVERSIONS_H
#define CONVERSIONS_H

namespace Conversions
{
    inline float toRpm(float unitsPer100ms) { return unitsPer100ms / 27.30667f; }
	inline float fromRpm(float rpm) { return rpm * 27.30667f; }
}

#endif