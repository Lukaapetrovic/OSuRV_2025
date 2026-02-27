
#pragma once


#define N_BUTTONS 4

// Mapiranje fizickih dugmadi (sta program vidi):
#define BUTTON_DESNO 1      // Fizicko dugme 2 = broj 1 = DESNO
#define BUTTON_LEVO  2      // Fizicko dugme 4 = broj 2 = LEVO
#define BUTTON_SREDINA 3    // Fizicko dugme 1 = broj 3 = SREDINA
static uint8_t buttons[N_BUTTONS];

static void print_buttons(const char* msg) {
	char readable_buttons[N_BUTTONS+1]; // '0' or '1' per button + null terminator
	for(int i = 0; i < N_BUTTONS; i++){
		readable_buttons[i] = buttons[i] ? '1' : '0';
	}
	readable_buttons[N_BUTTONS] = '\0';

	printf("%s: %s\n", msg, readable_buttons);
}
