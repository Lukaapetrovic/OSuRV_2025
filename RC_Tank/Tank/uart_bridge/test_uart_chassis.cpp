#include "SerialCPP.h"
#include <iostream>
#include <unistd.h>
#include <string>

/*
 * Expected byte format for the chassis motors:
 * SSHHTTMM
 *
 * S - Chassis speed
 * H - Turn hardness
 * T - Turn type
 * M - Move type
 *
 */

enum t_move {CM_IDLE = 0, CM_FULL_BRAKE, CM_FORWARD, CM_REVERSE};
enum t_turn {CT_STRAIGHT = 0, CT_RIGHT, CT_LEFT, CT_INVALID };
enum t_hardness { CTH_FLOAT = 0, CTH_BRAKE, CTH_IN_PLACE, CTH_INVALID };
enum t_speed { S_SLOW = 0, S_MEDIUM, S_FAST, S_INVALID };

std::string t_move_str[4] = 	{"CM_IDLE", "CM_FULL_BRAKE", "CM_FORWARD", "CM_REVERSE"};
std::string t_turn_str[4] = 	{"CT_STRAIGHT", "CT_RIGHT", "CT_LEFT", "CT_INVALID"};
std::string t_hardness_str[4] = {"CTH_FLOAT", "CTH_BRAKE", "CTH_IN_PLACE", "CTH_INVALID"};
std::string t_speed_str[4] = 	{"S_SLOW", "S_MEDIUM", "S_FAST", "S_INVALID"};

struct instruction {
	t_move move;
	t_turn turn;
	t_hardness hardness;
	t_speed speed;//systemctl status serial-getty@ttyAMA1.service
};

uint8_t parseInstruction(instruction inst) {
	uint8_t chassis_move = 		(uint8_t)inst.move;
	uint8_t chassis_turn = 		(uint8_t)inst.turn;
	uint8_t chassis_hardness = 	(uint8_t)inst.hardness;
	uint8_t chassis_speed = 	(uint8_t)inst.speed;

	return (chassis_speed << 6) | (chassis_hardness << 4) | (chassis_turn << 2) | chassis_move;
}

int main(int argc, char** argv) {
	SerialCPP::SerialCPP uart("/dev/ttyAMA0", SerialCPP::BaudRate::BR_115200);

	if (!uart.open())
	{
		std::cerr << "Failed to open /dev/ttyAMA0\n";
		return -1;
	}

	std::vector<instruction> instructions = {
		{CM_FORWARD, CT_STRAIGHT, CTH_FLOAT, S_FAST}	
	};

	std::vector<uint8_t> cmd;

	for(int i = 0; i < instructions.size(); i++) {
		std::cout << "Try to send command " << i << " ("
				  << t_move_str[instructions[i].move] << ", "
				  << t_turn_str[instructions[i].turn] << ", "
				  << t_hardness_str[instructions[i].hardness] << ", "
				  << t_speed_str[instructions[i].speed] <<
				  ")" << std::endl;
		uint8_t chassis = parseInstruction(instructions[i]);
		std::cout << chassis << std::endl;
		cmd.push_back(chassis);
	}
	uart.writeBytes(cmd);

	uart.close();
	return 0;
}

