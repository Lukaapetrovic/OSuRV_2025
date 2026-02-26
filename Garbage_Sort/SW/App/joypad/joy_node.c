
#include <stdio.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <zmq.h>
#include <errno.h>

#include "common.h"

#define ZMQ_ENDPOINT "tcp://0.0.0.0:5555"

int main() {
	// Initialize ZeroMQ context and PUSH socket
	void* context = zmq_ctx_new(); 
	if(!context){
		perror("Failed to create ZeroMQ context");
		return 1;
	}
	void* publisher = zmq_socket(context, ZMQ_PUB);
	if(!publisher){
		perror("Failed to create ZeroMQ PUB socket");
		zmq_ctx_destroy(context);
		return 1;
	}
	if(zmq_bind(publisher, ZMQ_ENDPOINT) != 0){
		perror("Failed to bind ZeroMQ PUB socket");
		zmq_close(publisher);
		zmq_ctx_destroy(context);
		return 1;
	}

	int js_fd;
	int num_of_axes = 0;
	int num_of_buttons = 0;

	// Open the joystick device file in read-only mode
	js_fd = open("/dev/input/js0", O_RDONLY);
	if(js_fd == -1){
		perror("Error opening joystick device");
		return 1;
	}

	ioctl(js_fd, JSIOCGAXES, &num_of_axes);
	ioctl(js_fd, JSIOCGBUTTONS, &num_of_buttons);

	if(num_of_buttons < N_BUTTONS){
		fprintf(stderr, "ERROR: Strange joystick with %d buttons! %d buttons are needed!\n", num_of_buttons, N_BUTTONS);
		return 1;
	}

	while(1){
		struct js_event js_event_data;

		if(read(js_fd, &js_event_data, sizeof(struct js_event)) != sizeof(struct js_event)){
			perror("Error reading joystick event");
			break;
		}

		if(js_event_data.type & JS_EVENT_BUTTON){
			if(js_event_data.number < N_BUTTONS){
				buttons[js_event_data.number] = js_event_data.value;
				
				print_buttons("joy_node publishing buttons");

				if(zmq_send(publisher, buttons, sizeof(buttons), 0) == -1){
					perror("Failed to publish buttons");
				}
			}
		 } 
	}

	close(js_fd);


	zmq_close(publisher);
	zmq_ctx_destroy(context);

	return 0;
}