
#include "pwm.h"


#include "hw_pwm.h"
#include "sw_pwm.h"
#include "include/motor_ctrl.h" // MOTOR_CLTR__N_SERVO
#include "include/motor_ctrl_pwm_map.h"

int pwm__init(void) {
	int r;
	
	r = hw_pwm__init();
	if(r){
		goto exit;
	}
	
	r = sw_pwm__init();
	if(r){
		goto exit;
	}

exit:
	if(r){
		pwm__exit();
	}
	return 0;
}

void pwm__exit(void) {
	sw_pwm__exit();
	hw_pwm__exit();
}


void pwm__set_moduo(pwm__ch_t ch, uint32_t moduo) {
	int s;
	if(ch >= MOTOR_CLTR__N_SERVO){
		return;
	}
	
	for(s = 0; s < N_PWM_SHARE; s++){
		switch(pwm_map[ACTIVE_PWM_MAP][ch][s].type){
			case HW:
				hw_pwm__set_moduo(
					pwm_map[ACTIVE_PWM_MAP][ch][s].ch,
					moduo
				);
				break;
			case SW:
				sw_pwm__set_moduo(
					pwm_map[ACTIVE_PWM_MAP][ch][s].ch,
					moduo
				);
				break;
			default:
				break;
		}
	}
}

void pwm__set_threshold(pwm__ch_t ch, uint32_t threshold) {
	int s;
	if(ch >= MOTOR_CLTR__N_SERVO){
		return;
	}
	
	for(s = 0; s < N_PWM_SHARE; s++){
		switch(pwm_map[ACTIVE_PWM_MAP][ch][s].type){
			case HW:
				hw_pwm__set_threshold(
					pwm_map[ACTIVE_PWM_MAP][ch][s].ch,
					threshold
				);
				break;
			case SW:
				sw_pwm__set_threshold(
					pwm_map[ACTIVE_PWM_MAP][ch][s].ch,
					threshold
				);
				break;
			default:
				break;
		}
	}
}
