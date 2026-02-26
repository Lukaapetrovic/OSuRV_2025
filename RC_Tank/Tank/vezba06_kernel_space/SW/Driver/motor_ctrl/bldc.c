
#include "bldc.h"

#include "gpio.h"

#include <linux/printk.h>
#include <linux/atomic.h>

#include <linux/gpio.h> // gpio stuff
#include <linux/interrupt.h> // irq stuff


typedef struct {
	uint8_t dir_gpio_no;
	uint8_t pg_gpio_no;
	const char* pg_label;
	int pg_irq;
	volatile dir_t dir;
	atomic64_t pulse_cnt;
} bldc_t;
static bldc_t bldc[] = {
	{
		6, // GPIO6, pin 31
		5, // GPIO5, pin 29
		"irq_gpio5",
		CW,
		0
	}
};


int bldc__init(void) {
	int r;
	bldc__ch_t ch;
	
	for(ch = 0; ch < BLDC__N_CH; ch++){
		gpio__steer_pinmux(bldc[ch].dir_gpio_no, GPIO__OUT);
		bldc__set_dir(ch, CW);
		
		//TODO Init timer
		
		//TODO Init IRQ
	}
	
	
	return 0;
}

void bldc__exit(void) {
	bldc__ch_t ch;
	
	for(ch = 0; ch < BLDC__N_CH; ch++){
		gpio_free(bldc[ch].pg_gpio_no);
	}
}


void bldc__set_dir(bldc__ch_t ch, dir_t dir) {
	if(ch >= BLDC__N_CH){
		return;
	}
	bldc[ch].dir = dir;
	if(dir == CW){
		gpio__set(bldc[ch].dir_gpio_no);
	}else{
		gpio__clear(bldc[ch].dir_gpio_no);
	}
}

void bldc__set_duty(bldc__ch_t ch, u16 duty_permille) {
	if(ch >= BLDC__N_CH){
		return;
	}
	// TODO For SW PWM @ GPIO24, pin 18
}

void bldc__get_pulse_cnt(bldc__ch_t ch, int64_t* pulse_cnt) {
	if(ch >= BLDC__N_CH){
		return;
	}
	// TODO For pulse count @ GPIO5, pin 29
}
