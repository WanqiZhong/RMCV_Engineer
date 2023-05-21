#include "data.hpp"
// #include <fmt/format.h>

MODE cast_run_mode(uint8_t mode)
{
	switch (mode)
	{
	case 0:
		return GoldMode;
	case 1:
		return SilverMode;
	case 2:
		return ExchangeSiteMode;
	case 3:
		return HALT;
	default:
		return Unknown;
	}
}

