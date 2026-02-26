#include <stdio.h>
#include <zmq.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

#include "common.h"

// Hostname/IP of publisher (joy_node or routine_node).
#define ZMQ_ENDPOINT "tcp://127.0.0.1:5555"

#define DEV_FN "/dev/gpio_stream"


// GPIO pinovi
#define MOTOR_EN 2    //ukljuci iskljuci motor
#define MOTOR_PIN3 3	// kontrola smera - levo
#define MOTOR_PIN4 4	// kontrola smera - desno
#define SENSOR_SREDINA 22
#define SENSOR_LEVO 23
#define SENSOR_DESNO 24

// Pozicije
#define POS_NEPOZNATO 0
#define POS_LEVO 1
#define POS_SREDINA 2
#define POS_DESNO 3

// Globalne promenljive za stanje
int current_position = POS_NEPOZNATO;
int target_position = POS_NEPOZNATO;
int motor_running = 0;

int gpio_write(int fd, uint8_t pin, uint8_t value) {
	uint8_t pkg[3];
	pkg[0] = 'w';
	pkg[1] = pin;
	pkg[2] = value;

	if(write(fd, &pkg, 3) != 3){
		perror("Failed to write to GPIO");
		return -1;
	}
	return 0;
}

int gpio_read(int fd, uint8_t pin) {
	uint8_t cmd[2];
	cmd[0] = 'r';
	cmd[1] = pin;
	
	if(write(fd, cmd, 2) != 2){
		perror("Failed to write read command to GPIO");
		return -1;
	}
	
	usleep(100); 
	
	uint8_t value = 0;
	if(read(fd, &value, 1) != 1){
		perror("Failed to read from GPIO");
		return -1;
	}
	
	return value;
}

void motor_stop(int gpio_fd) {
	gpio_write(gpio_fd, MOTOR_EN, 0);
	motor_running = 0;
	printf("Motor ZAUSTAVLJEN\n");
}

void motor_start_cw(int gpio_fd) {
	// Smer kazaljke (CW): pin4=1, pin3=0
	gpio_write(gpio_fd, MOTOR_PIN4, 1);
	gpio_write(gpio_fd, MOTOR_PIN3, 0);
	gpio_write(gpio_fd, MOTOR_EN, 1);
	motor_running = 1;
	printf("Motor pokrenut CW (ka DESNO)\n");
}

void motor_start_ccw(int gpio_fd) {
	// Suprotno od kazaljke (CCW): pin4=0, pin3=1
	gpio_write(gpio_fd, MOTOR_PIN4, 0);
	gpio_write(gpio_fd, MOTOR_PIN3, 1);
	gpio_write(gpio_fd, MOTOR_EN, 1);
	motor_running = 1;
	printf("Motor pokrenut CCW (ka LEVO)\n");
}

// Provera trenutne pozicije na osnovu senzora
int check_current_position(int gpio_fd) {
	
	int sensor_levo = gpio_read(gpio_fd, SENSOR_LEVO);
	int sensor_sredina = gpio_read(gpio_fd, SENSOR_SREDINA);
	int sensor_desno = gpio_read(gpio_fd, SENSOR_DESNO);
	
	if(sensor_levo > 0) {
		printf("Pritisnut prekidac levo-pin:23.\n");
		return POS_LEVO;
	}
	if(sensor_sredina > 0) {
		printf("Pritisnut prekidac srednji-pin:22.\n");
		return POS_SREDINA;
	}
	if(sensor_desno > 0) {
		printf("Pritisnut prekidac desni-pin:24.\n");
		return POS_DESNO;
	}
	
	return POS_NEPOZNATO;
}

// Pokretanje motora ka cilju
void move_to_target(int gpio_fd) {
	if(current_position == target_position) {
		// Vec smo na cilju
		if(motor_running) {
			motor_stop(gpio_fd);
		}
		return;
	}
	
	printf("Pomeranje: %d -> %d\n", current_position, target_position);
	
	// Određivanje smera kretanja
	if(target_position == POS_DESNO) {
		// Idemo ka DESNO
		motor_start_cw(gpio_fd);
	}
	else if(target_position == POS_LEVO) {
		// Idemo ka LEVO
		motor_start_ccw(gpio_fd);
	}
	else if(target_position == POS_SREDINA) {
		// Idemo ka SREDINI
		if(current_position == POS_LEVO) {
			motor_start_cw(gpio_fd); // Sa LEVO ka SREDINA je CW
		} else if(current_position == POS_DESNO) {
			motor_start_ccw(gpio_fd); // Sa DESNO ka SREDINA je CCW
		} else {
			// Ako ne znamo gde smo, idemo najpre ka sredini u CW smeru
			motor_start_cw(gpio_fd);
		}
	}
}

const char* position_name(int pos) {
	switch(pos) {
		case POS_LEVO: 
			return "LEVO";
		case POS_SREDINA: 
			return "SREDINA";
		case POS_DESNO: 
			return "DESNO";
		default: 
			return "NEPOZNATO";
	}
}
int main() {
//Otvaranje GPIO drajver
	int gpio_fd = open(DEV_FN, O_RDWR);
	if(gpio_fd < 0){
		perror("Failed to open GPIO driver");
		return 1;
	}
//Inicijalizacija ZeroMQ
	void* context = zmq_ctx_new();
	if(!context){
		perror("Failed to create ZeroMQ context");
		return 1;
	}
	void* subscriber = zmq_socket(context, ZMQ_SUB);
	if(!subscriber){
		perror("Failed to create ZeroMQ socket");
		zmq_ctx_destroy(context);
		return 1;
	}
	if(zmq_connect(subscriber, ZMQ_ENDPOINT) != 0){
		perror("Failed to connect ZeroMQ socket");
		zmq_close(subscriber);
		zmq_ctx_destroy(context);
		return 1;
	}
	if(zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0) != 0){  
		perror("Failed to set ZMQ_SUBSCRIBE");
		zmq_close(subscriber);
		zmq_ctx_destroy(context);
		return 1;
	}

	printf("Connected and listening on %s...\n", ZMQ_ENDPOINT);
	
	  motor_stop(gpio_fd);
	
	// Provera pocetne pozicije - da znamo u prvoj iteraciji gde se nalazimo
	current_position = check_current_position(gpio_fd);
	printf("Pocetna pozicija: %s\n", position_name(current_position));

	/*
	currect_position - prethodna pozicija
	new_position -  pozicija u ovom trenutku
	*/

	while(1){
		zmq_msg_t msg;
		zmq_msg_init(&msg);
		int bytes = zmq_msg_recv(&msg, subscriber, ZMQ_DONTWAIT); // Non-blocking receive.
		if(bytes == N_BUTTONS){
			memcpy(buttons, zmq_msg_data(&msg), bytes); 

			print_buttons("wiper_node recv buttons");

		// Postavljanje ciljne pozicije na osnovu pritisnutog dugmeta
			if(buttons[BUTTON_SREDINA]) {
				if(target_position != POS_SREDINA) {
					printf("Korisnik je pritisnuo: SREDINA \n");
					target_position = POS_SREDINA;
				}
			}
			else if(buttons[BUTTON_DESNO]) {
				if(target_position != POS_DESNO) {
					printf("Korisnik je pritisnuo: DESNO \n");
					target_position = POS_DESNO;
				}
			}
			else if(buttons[BUTTON_LEVO]) {
				if(target_position != POS_LEVO) {
					printf("Korisnik je pritisnuo: LEVO \n");
					target_position = POS_LEVO;
				}
			}
		}
		zmq_msg_close(&msg);
		
		// Provera trenutne pozicije
		int new_position = check_current_position(gpio_fd);
		
		// Ažuriranje pozicije ako smo na senzoru
		if(new_position != POS_NEPOZNATO) {
			if(new_position != current_position) {

				current_position = new_position;
				printf("Nova pozicija: %s\n", position_name(current_position));
			
				if(motor_running) {
					printf("Stigao na senzor %s\n", position_name(current_position));
   					motor_stop(gpio_fd);		
				}	
			
			}
			
			// Ako smo stigli na cilj, zaustavi motor
			if(current_position == target_position && motor_running) {
				printf("STIGLI SMO NA CILJ: %s\n", position_name(current_position));
				motor_stop(gpio_fd);
			}
		}
		
		// Ako nismo na cilju, nastavi kretanje
		if(target_position != POS_NEPOZNATO && current_position != target_position && !motor_running) {
			move_to_target(gpio_fd);
		}
		
		usleep(10000);
	}
    motor_stop(gpio_fd);  
	zmq_close(subscriber);
	zmq_ctx_destroy(context);
	
	close(gpio_fd);

	return 0;
}

