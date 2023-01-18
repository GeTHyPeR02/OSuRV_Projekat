
#include <stdint.h> // uint16_t and family
#include <stdbool.h> // bool
#include <stdio.h> // printf and family
#include <string.h> // strerror()
#include <unistd.h> // file ops
#include <fcntl.h> // open() flags
#include <sys/ioctl.h> // ioctl()

#include "motor_ctrl.h"


void usage(FILE* f){
	fprintf(f,
"\nUsage: "\
"\n	test_bldc -h|--help"\
"\n		print this help i.e."\
"\n	test_bldc speed"\
"\n	speed = [-100, 100]"\
"\n"
	);
}

static inline int c_str_eq(const char* a, const char* b) {
	return !strcmp(a, b);
}

int parse_args(
	int argc,
	char** argv,
	int* p_speed
) {
	int n;
	if(argc == 2){
		if(c_str_eq(argv[1], "-h") || c_str_eq(argv[1], "--help")){
			// Print help.
			usage(stdout);
			return 0;
		}else{
			n = sscanf(argv[1], "%d", p_speed);
			if(n != 1){
				fprintf(stderr, "ERROR: Invalid number \"%s\"!\n", argv[1]);
				return 3;
			}
		}
	}else{
		// Error.
		fprintf(stderr, "ERROR: Wrong number of arguments!\n");
		usage(stderr);
		return 1;
	}
	
	return 0;
}


int main(int argc, char** argv){
	int speed;
	int r = parse_args(argc, argv, &speed);
	if(r){
		return r;
	}
	
	printf("speed = %d\n", speed);
	// Limits.
	if(!(-100 <= speed && speed <=  100)){
		fprintf(stderr, "ERROR: speed out of range [0, 100]!\n");
		usage(stderr);
		return 2;
	}

	
	int fd;
	fd = open(DEV_FN, O_RDWR);
	if(fd < 0){
		fprintf(stderr, "ERROR: \"%s\" not opened!\n", DEV_FN);
		fprintf(stderr, "fd = %d %s\n", fd, strerror(-fd));
		return 4;
	}
	
	const uint16_t moduo = 20; // 5kHz
	printf("moduo = %d\n", moduo);
	
	motor_ctrl__ioctl_arg_moduo_t ia;
	ia.ch = 0;
	ia.moduo = moduo;
	r = ioctl(fd, IOCTL_MOTOR_CLTR_SET_MODUO, *(unsigned long*)&ia);
	if(r){
		fprintf(stderr, "ERROR: ioctl went wrong returning %d!\n", r);
		return 4;
	}
	
	int16_t duty[2];
	
	// |speed| = [0, 100] -> threshold = [10, 0].
	// it will be <<1 in write() so then will be [20, 0].
	uint8_t abs_speed;
	uint16_t threshold;
	if(speed > 0){
		abs_speed = speed;
		threshold = (100-abs_speed)/10;
		duty[0] = threshold;
	}else{
		abs_speed = -speed;
		threshold = (100-abs_speed)/10;
		duty[0] = -threshold;
	}
	
	printf("threshold = %d\n", threshold);
	printf("duty = %d\n", duty[0]);
	
	// Write just channel 0, which is BLDC.
	// Channel 1 is servo and here is not changed.
	int s = sizeof(duty[0])*1;
	r = write(fd, (char*)&duty, s);
	if(r != s){
		fprintf(stderr, "ERROR: write went wrong!\n");
		return 4;
	}
	
	motor_ctrl__read_arg_fb_t ra;
	r = read(fd, (char*)&ra, sizeof(ra));
	if(r != sizeof(ra)){
		fprintf(stderr, "ERROR: read went wrong!\n");
		return 4;
	}
	
	int64_t pulse_cnt_fb = ra.pulse_cnt_fb[0];
	printf("pulse_cnt_fb = %lld\n", (long long int)pulse_cnt_fb);
	
	
	close(fd);
	
	printf("End.\n");
	
	return 0;
}
