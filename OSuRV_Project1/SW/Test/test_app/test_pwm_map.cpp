
#include <iostream>
using namespace std;

#include "motor_ctrl_pwm_map.h"


#define DEBUG(x) do{ cout << #x << " = " << x << endl; }while(0)

ostream& operator<<(ostream& os, const pwm_map_elem_t& e) {
	switch(e.type){
		case NONE:
			os << "NONE";
			break;
		case HW:
			os << "HW";
			break;
		case SW:
			os << "SW";
			break;
		default:
			os << "???";
			break;
	}
	os << (int)e.ch;
	return os;
}

int main() {
	for(int m = 0; m < N_PWM_MAP; m++){
		for(int c = 0; c < N_PWM_CH; c++){
			cout << pwm_map[m][c][0];
			for(int s = 1; s < N_PWM_SHARE; s++){
				if(pwm_map[m][c][s].type != NONE){
					cout << "|";
					cout << pwm_map[m][c][s];
				}
			}
			cout << "\t";
		}
		cout << endl;
	}
	
	return 0;
}