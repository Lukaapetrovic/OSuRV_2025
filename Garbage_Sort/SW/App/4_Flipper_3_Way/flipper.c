
#include <stdint.h> // uint16_t and family
#include <stdio.h> // printf and family
#include <unistd.h> // file ops
#include <fcntl.h> // open() flags
#include <string.h> // strerror()
#include <errno.h> // errno
#include <signal.h>	//za preide
#include <stdlib.h>

#define DEV_STREAM_FN "/dev/gpio_stream"

int fd;


int main()
{
	int r;
	int smer_kazaljke = 1;
	
	fd = open(DEV_STREAM_FN, O_RDWR);
	if(fd < 0){
		fprintf(stderr, "ERROR: \"%s\" not opened!\n", DEV_STREAM_FN);
		fprintf(stderr, "fd = %d %s\n", fd, strerror(-fd));
		return 4;
	}
	

	// ccw
	uint8_t set_p4_0[3] = {'w', 4, 0}; 
	uint8_t set_p3_1[3] = {'w', 3, 1};		
			
	// cw obrnut smer
	uint8_t set_p4_1[3] = {'w', 4, 1}; 
	uint8_t set_p3_0[3] = {'w', 3, 0};								
	
	uint8_t motor_start[3] = {'w', 2, 1};	
	uint8_t motor_stop[3] = {'w', 2, 0};			
	
	uint8_t read_levi[2] = {'r', 23};			
	uint8_t read_desni[2] = {'r',24};		
	uint8_t read_srednji[2] = {'r', 22};		
	
	
	r = write(fd, set_p4_1, sizeof(set_p4_1));
	if(r != sizeof(set_p4_1)){
		fprintf(stderr, "ERROR: write went wrong!\n");
		return 4;
	}
	usleep(100000);

	printf("Stigli smo do ovde : 1\n");

	r = write(fd, set_p3_0, sizeof(set_p3_0));
	if(r != sizeof(set_p3_0)){
		fprintf(stderr, "ERROR: write went wrong!\n");
		return 4;
	}
	usleep(100000);

	printf("Stigli smo do ovde: 2\n");

	r = write(fd, motor_start, sizeof(motor_start));
	if(r != sizeof(motor_start)){
		fprintf(stderr, "ERROR: write went wrong!\n");
		return 4;
	}
	usleep(100000); 


	printf("Stigli smo do ovde : 3\n");

	uint8_t rd_val=0;			// levi
	uint8_t rd_val2 = 0;		//desni
	uint8_t rd_val3 = 0;		//srednji

	while(1)
	{	
		//lijevi prekidac
		r = write(fd, read_levi, sizeof(read_levi));
		if(r != sizeof(read_levi)){
			fprintf(stderr, "ERROR: write went wrong!\n");
			return 4;
		}
		usleep(100); 

		r = read(fd, (char*)&rd_val, sizeof(rd_val));
		if(r != sizeof(rd_val)){
			fprintf(stderr, "ERROR: read went wrong!\n");
			return 5;
		}
		
		//desni prekidac
		r = write(fd, read_desni, sizeof(read_desni));
		if(r != sizeof(read_desni)){
			fprintf(stderr, "ERROR: write went wrong!\n");
			return 4;
		}
		usleep(100); 

		r = read(fd, (char*)&rd_val2, sizeof(rd_val2));
		if(r != sizeof(rd_val2)){
			fprintf(stderr, "ERROR: read went wrong!\n");
			return 5;
		}
		
		//srednji
		r = write(fd, read_srednji, sizeof(read_srednji));
		if(r != sizeof(read_srednji)){
			fprintf(stderr, "ERROR: write went wrong!\n");
			return 4;
		}
		usleep(100); 
		

		r = read(fd, (char*)&rd_val3, sizeof(rd_val3));
		if(r != sizeof(rd_val3)){
			fprintf(stderr, "ERROR: read went wrong!\n");
			return 5;
		}
		
	//provjera
	if(rd_val || rd_val2 || rd_val3){
		write(fd, motor_stop, sizeof(motor_stop));
		
		//lijevi
		if(rd_val){		
			printf("Levi prekidac aktivan!\n");
			write(fd, set_p4_1, sizeof(set_p4_1));
			write(fd, set_p3_0, sizeof(set_p3_0));
			smer_kazaljke = 1;
			
		}
		if(rd_val2){
			printf("Desni prekidac aktivan!\n");
			
			if(smer_kazaljke){
				r = write(fd, set_p4_0, sizeof(set_p4_0));
				if(r != sizeof(set_p4_0)){
				fprintf(stderr, "ERROR: write went wrong!\n");
				}
				
				r = write(fd, set_p3_1, sizeof(set_p3_1));
				if(r != sizeof(set_p3_1)){
				fprintf(stderr, "ERROR: write went wrong!\n");
				}
				
				smer_kazaljke = 0;
			} else {
				r = write(fd, set_p4_0, sizeof(set_p4_0));
				if(r != sizeof(set_p4_0)){
				fprintf(stderr, "ERROR: write went wrong!\n");
				return 4;
				}
				
				r = write(fd, set_p3_1, sizeof(set_p3_1));
				if(r != sizeof(set_p3_1)){
				fprintf(stderr, "ERROR: write went wrong!\n");
				return 4;
				}
				
				smer_kazaljke = 1;
			}
		}
		
		if(rd_val3){
			printf("Srednji prekidac aktivan!\n");
		}
		
		printf("\nCekam 2 sekunde...\n");
		sleep(2);
						
		//ponovo pokretanje motora
		printf("\n Proslo dve sekunde motor ponovo rece.\n");
		write(fd, motor_start, sizeof(motor_start));
		usleep(500000);
			
	}

}
	close(fd);

	return 0;
}

