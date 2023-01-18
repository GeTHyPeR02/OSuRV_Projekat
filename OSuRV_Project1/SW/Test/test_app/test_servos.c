
#include <stdint.h> // uint16_t and family
#include <stdio.h> // printf and family
#include <unistd.h> // file ops
#include <fcntl.h> // open() flags
#include <string.h> // strerror()
#include <errno.h> // errno

#include "motor_ctrl.h"


void usage(FILE* f){
	fprintf(f,
"\nUsage: "\
"\n	test_servos -h|--help"\
"\n		print this help i.e."\
"\n	test_servos w servo_idx duty"\
"\n		write angle at servo_idx servo motor"
"\n	test_servos r servo_idx"\
"\n		read feedback angle of servo_idx servo motor"\
"\n	servo_idx = [0, 6)"\
"\n	duty = [0, 1000]"\
"\n"
	);
}

static inline int c_str_eq(const char* a, const char* b) {
	return !strcmp(a, b);
}

int parse_args(
	int argc,
	char** argv,
	char* p_op,
	int* p_servo_idx,
	int* p_angle
) {
	if(argc == 2){
		if(c_str_eq(argv[1], "-h") || c_str_eq(argv[1], "--help")){
			// Print help.
			usage(stdout);
			return 0;
		}else{
			// Error.
			fprintf(stderr, "ERROR: Wrong argument \"%s\"!\n", argv[1]);
			usage(stderr);
			return 1;
		}
	}else if(argc == 3){
		if(!c_str_eq(argv[1], "r")){
			fprintf(stderr, "ERROR: Wrong command \"%s\"!\n", argv[1]);
			usage(stderr);
			return 2;
		}
		*p_op = 'r';
		int n;
		n = sscanf(argv[2], "%d", p_servo_idx);
		if(n != 1){
			fprintf(stderr, "ERROR: Invalid number \"%s\"!\n", argv[2]);
			return 3;
		}
	}else if(argc == 4){
		if(!c_str_eq(argv[1], "w")){
			fprintf(stderr, "ERROR: Wrong command \"%s\"!\n", argv[1]);
			usage(stderr);
			return 2;
		}
		*p_op = 'w';
		int n;
		n = sscanf(argv[2], "%d", p_servo_idx);
		if(n != 1){
			fprintf(stderr, "ERROR: Invalid number \"%s\"!\n", argv[2]);
			return 3;
		}
		n = sscanf(argv[3], "%d", p_angle);
		if(n != 1){
			fprintf(stderr, "ERROR: Invalid number \"%s\"!\n", argv[3]);
			return 3;
		}
	}else{
		// Error.
		fprintf(stderr, "ERROR: Wrong number of arguments!\n");
		usage(stderr);
		return 1;
	}
	//TODO limits
	return 0;
}


int main(int argc, char** argv){
	char op;
	int servo_idx;
	int duty;
	int r = parse_args(argc, argv, &op, &servo_idx, &duty);
	if(r){
		return r;
	}

	uint16_t duties[MOTOR_CLTR__N_SERVO] = {0};
	
	int fd;
	fd = open(DEV_FN, O_RDWR);
	if(fd < 0){
		fprintf(stderr, "ERROR: \"%s\" not opened!\n", DEV_FN);
		fprintf(stderr, "fd = %d %s\n", fd, strerror(-fd));
		return 4;
	}
	
	
	if(op == 'w'){
		printf("duty = %d\n", duty);
		duties[servo_idx] = duty; // [permilles]
		
		for(int i = 0; i < MOTOR_CLTR__N_SERVO; i++){
			printf("duties[%d] = %d\n", i, duties[i]);
		}
		
		r = write(fd, (char*)&duties, sizeof(duties));
		if(r != sizeof(duties)){
			fprintf(stderr, "ERROR: write went wrong!\n");
			return 4;
		}
	}else if(op == 'r'){
		r = read(fd, (char*)&duties, sizeof(duties));
		if(r != sizeof(duties)){
			fprintf(stderr, "ERROR: read went wrong!\n");
			return 4;
		}
		for(int i = 0; i < MOTOR_CLTR__N_SERVO; i++){
			printf("duties[%d] = %d\n", i, duties[i]);
		}
		
		duty = duties[servo_idx]; // [permilles]
		printf("duty = %d\n", duty);
	}

	close(fd);

	printf("End.\n");

	return 0;
}
