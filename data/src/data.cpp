#include "data.hpp"

uint8_t get_simple_id(uint8_t id)
{
    // ID=color(0~3:B R N P)*9+type(0~8:0 1 2 3 4 5 O Bs Bb)

	constexpr uint8_t offset = 9;
	switch (id)
	{
	case Robot_id_dji::BLUE_SENTRY:
		return 0 + 0;
	case Robot_id_dji::BLUE_HERO:
		return 0 + 1;
	case Robot_id_dji::BLUE_ENGINEER:
		return 0 + 2;
	case Robot_id_dji::BLUE_STANDARD_1:
		return 0 + 3;
	case Robot_id_dji::BLUE_STANDARD_2:
		return 0 + 4;
	case Robot_id_dji::BLUE_STANDARD_3:
		return 0 + 5;
	case Robot_id_dji::BLUE_AERIAL:
		return 0 + 6;
	case Robot_id_dji::RED_SENTRY:
		return offset + 0;
	case Robot_id_dji::RED_HERO:
		return offset + 1;
	case Robot_id_dji::RED_ENGINEER:
		return offset + 2;
	case Robot_id_dji::RED_STANDARD_1:
		return offset + 3;
	case Robot_id_dji::RED_STANDARD_2:
		return offset + 4;
	case Robot_id_dji::RED_STANDARD_3:
		return offset + 5;
	case Robot_id_dji::RED_AERIAL:
		return offset + 6;
	default: // Unknown
		return UNKNOWN_ID;
	}
}

MODE cast_run_mode(uint8_t mode)
{
	switch (mode)
	{
	case 0:
	case 1:
		return AUTO_AIM;
		break;
	case 2:
		return S_WM;
	case 3:
		return B_WM;
	default:
		return Unknown;
	}
}

// std::string IMU_DATA_MSG::reprln()const
// {
// 	return fmt::format(
// 		"{:.3f} {:.3f} {:.3f} {} {} {:.3f} {:.5f}\n",
// 		imu.roll, imu.pitch, imu.yaw,
// 		mode, id, v,
// 		time_stamp
// 	);
// }