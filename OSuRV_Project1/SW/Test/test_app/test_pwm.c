
#include <stdint.h> // uint16_t and family
#include <stdio.h> // printf and family
#include <unistd.h> // file ops
#include <fcntl.h> // open() flags
#include <string.h> // strerror()
#include <errno.h> // errno

#include "motor_ctrl.h"


int main(int argc, char** argv) {
	int r;
	
	uint16_t duties[MOTOR_CLTR__N_SERVO] = {
		70,
		80,
		90,
		100,
		110,
		120
	};
	
	int fd;
	fd = open(DEV_FN, O_RDWR);
	if(fd < 0){
		fprintf(stderr, "ERROR: \"%s\" not opened!\n", DEV_FN);
		fprintf(stderr, "fd = %d %s\n", fd, strerror(-fd));
		return 4;
	}
	
	
	for(int i = 0; i < MOTOR_CLTR__N_SERVO; i++){
		printf("duties[%d] = %d\n", i, duties[i]);
	}
	
	r = write(fd, (char*)&duties, sizeof(duties));
	if(r != sizeof(duties)){
		fprintf(stderr, "ERROR: write went wrong!\n");
		return 4;
	}

	close(fd);

	printf("End.\n");

	return 0;
}
