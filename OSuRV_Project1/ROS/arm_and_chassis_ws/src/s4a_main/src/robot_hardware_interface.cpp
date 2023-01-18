
#include "robot_hardware_interface.hpp"

// Config.
#define LOOP_HZ 25
// Speed is 0.1s for 90 deg.
#define SPEED (90*M_PI/180/0.1) // [rad/s]

#define DEV_FN "/dev/motor_ctrl"
#define N_CH_OFFSET 2


// Includes for driver.
#include <string.h> // strerror()
#include <unistd.h> // file ops
#include <fcntl.h> // open() flags


#include <algorithm>
using namespace std;

#include "ostream_coloring.hpp"
using namespace ostream_color_log;
#include "exec.hpp"

#include <angles/angles.h>
#include <boost/scoped_ptr.hpp>


#define DEBUG(x) do{ ROS_DEBUG_STREAM(#x << " = " << x); }while(0)

template<typename T>
static T sym_clamp(T x, T limit) {
	return clamp(x, -limit, limit);
}

RobotHardwareInterface::RobotHardwareInterface(ros::NodeHandle& nh) :
	nh(nh),
	first_print(true),
	n_joints(0), 
	log_joint_states(false),
	all_motors_sim(false),
	drv_fd(-1),
	offset_channels(false),
	motors_en(true)
{
	ros::param::param("~log_joint_states", log_joint_states, false);
	ROS_INFO(
		"log_joint_states = %s",
		log_joint_states ? "true" : "false"
	);
	
	// Open log file.
	string joints_log_fn = exec("roslaunch-logs");
	joints_log_fn.pop_back(); // Remove new line.
	joints_log_fn += "/robot_hardware_interface__joints.log";
	joints_log.open(joints_log_fn);
	if(!joints_log.is_open()){
		ROS_WARN(
			"Cannot open log file \"%s\"!",
			joints_log_fn.c_str()
		);
	}
	
	readJointNames();
	
	curr = State(n_joints);
	prev = State(n_joints);
	tmp_duty.resize(n_joints);
	acc_msg.data.resize(n_joints);
	
	
	ros::param::param("~all_motors_sim", all_motors_sim, false);
	ROS_INFO(
		"all_motors_sim = %s",
		all_motors_sim ? "true" : "false"
	);
	ros::param::param("~offset_channels", offset_channels, false);
	ROS_INFO(
		"offset_channels = %s",
		offset_channels ? "true" : "false"
	);
	
	
	if(!all_motors_sim){
		// Open driver.
		drv_fd = open(DEV_FN, O_RDWR);
		if(drv_fd < 0){
			ROS_WARN(
				"\"%s\" not opened! drv_fd = %d -> %s",
				DEV_FN, drv_fd, strerror(-drv_fd)
			);
		}
	}
	
	
	for(int i = 0; i < n_joints+1; ++i){ // +1 for symetrical finger
		hardware_interface::JointStateHandle jsh(
			joint_names[i],
			&curr.pos_fb[i],
			&curr.vel_fb[i],
			&curr.eff_fb[i]
		);
		fb_if.registerHandle(jsh);

		hardware_interface::JointHandle jh(jsh, &curr.pos_cmd[i]);

		cmd_if.registerHandle(jh);
	}
	registerInterface(&fb_if);
	registerInterface(&cmd_if);
	
	
	ctrl_manager = new controller_manager::ControllerManager(this, nh);
	ros::Duration timer_period = ros::Duration(1.0/LOOP_HZ);
	non_realtime_loop_timer = nh.createTimer(
		timer_period,
		&RobotHardwareInterface::update,
		this
	);
	
	motors_en_sub = nh.subscribe(
		"motors_en",
		1,
		&RobotHardwareInterface::motors_en_cb,
		this
	);
	
	acc_pub = nh.advertise<std_msgs::Float64MultiArray>(
		"acc",
		1
	);
}

RobotHardwareInterface::~RobotHardwareInterface() {
	if(!all_motors_sim){
		// Close driver.
		ROS_INFO("closing drv_fd");
		close(drv_fd);
	}
	delete ctrl_manager;
}

RobotHardwareInterface::State::State(size_t n_joints) {
	const double init_val = 0;
	pos_cmd.resize(n_joints+1, init_val);
	eff_fb.resize(n_joints+1, init_val);
	pos_fb.resize(n_joints+1, init_val);
	vel_fb.resize(n_joints+1, init_val);
	acc.resize(n_joints, init_val);
}

bool operator!=(
	const RobotHardwareInterface::State& a,
	const RobotHardwareInterface::State& b
) {
	return
		a.pos_cmd != b.pos_cmd ||
		a.eff_fb != b.eff_fb ||
		a.pos_fb != b.pos_fb ||
		a.vel_fb != b.vel_fb ||
		a.acc != b.acc;
}

#define TAB "  "
#define W 10
void RobotHardwareInterface::print_array(
	std::ostream& os,
	const char* name,
	const std::vector<double>& curr,
	const std::vector<double>& prev,
	double scale
) {
#if 1
	os << reset;
	os << "prev " << name << TAB;
	for(int id = 0; id < n_joints; id++){
		os << setw(W) << scale*prev[id];
		os << TAB;
	}
	os << endl;
#endif
	
	os << bold;
	os << "curr " << name << TAB;
	for(int id = 0; id < n_joints; id++){
		if(curr[id] == prev[id]){
			os << yellow;
		}else if(curr[id] > prev[id]){
			os << red;
		}else{ // if(curr[id] < prev[id]){
			os << blue;
		}
		os << setw(W) << scale*curr[id];
		os << TAB;
	}
	os << endl;
}

void RobotHardwareInterface::log_joint_states_fun(const ros::Time& t) {
	if(log_joint_states){
		if(curr != prev || first_print){
			first_print = false;
			std::ios old_state(nullptr);
			ostream& jl = joints_log;
			old_state.copyfmt(jl);
			jl << fixed << setprecision(5);
			
			jl << reset;
			jl << bold;
			jl << "     t              " << TAB << t.toSec() << endl;
			
			print_array(jl, "pos_cmd[deg]   ", curr.pos_cmd    , prev.pos_cmd   , 180/M_PI);
			print_array(jl, "pos_cmd[rad]   ", curr.pos_cmd    , prev.pos_cmd   );
			print_array(jl, "eff_fb         ", curr.eff_fb     , prev.eff_fb    );
			print_array(jl, "pos_fb[deg]    ", curr.pos_fb     , prev.pos_fb    , 180/M_PI);
			print_array(jl, "pos_fb[rad]    ", curr.pos_fb     , prev.pos_fb    );
			print_array(jl, "vel_fb[rad/s]  ", curr.vel_fb     , prev.vel_fb    );
			print_array(jl, "acc[rad/s^2]   ", curr.acc        , prev.acc       );
			
			jl << reset << endl;
			jl.copyfmt(old_state);
		}
	}
}

void RobotHardwareInterface::seek() {
	const int o = sizeof(tmp_duty[0])*N_CH_OFFSET*offset_channels;
	int r = lseek(drv_fd, SEEK_SET, o);
	if(r != o){
		if(r < 0){
			ROS_WARN(
				"lseek failed! errno = %d -> %s",
				errno, strerror(errno)
			);
		}else{
			ROS_WARN(
				"lseek failed! offset is %d but expected %d",
				r, o
			);
		}
	}
}

void RobotHardwareInterface::update(const ros::TimerEvent& e) {
	
	ros::Time now = ros::Time::now();
	ros::Duration elapsed_time = ros::Duration(e.current_real - e.last_real);
	read(elapsed_time);
	ctrl_manager->update(now, elapsed_time);
	write(elapsed_time);

	log_joint_states_fun(now);
	
	prev = curr;
}

void RobotHardwareInterface::read(ros::Duration elapsed_time) {
	double dt = elapsed_time.toSec();
	
	
	if(all_motors_sim){
		for(int id = 0; id < n_joints; id++){
			// rad = % * s * rad/s
			curr.pos_fb[id] += prev.eff_fb[id]/100*dt*SPEED;
		}
	}else{
		// Read from driver.
		
		seek();
		
		const int s = sizeof(tmp_duty[0])*n_joints; // Symetrical not read.
		int r = ::read(drv_fd, (char*)tmp_duty.data(), s);
		if(r != s){
			ROS_WARN("read went wrong!");
		}
		for(int id = 0; id < n_joints; id++){
			curr.pos_fb[id] = (tmp_duty[id] - 25)*(M_PI/100) - M_PI_2;
		}
	}
	
	for(int id = 0; id < n_joints; id++){
		double pos_cmd;
		if(motors_en){
			pos_cmd = curr.pos_cmd[id];
		}else{
			pos_cmd = 0;
		}
		// Estimate effort.
		curr.eff_fb[id] = sym_clamp(
			// Distance left / speed = time left
			// time left / dt * 100 = percent left.
			(pos_cmd - curr.pos_fb[id])/SPEED/dt * 100,
			100.0
		);
		curr.vel_fb[id] = (curr.pos_fb[id] - prev.pos_fb[id])/dt;
		curr.acc[id] = (curr.vel_fb[id] - prev.vel_fb[id])/dt;
	}
	
	// Symetrical finger.
	curr.eff_fb[n_joints] = curr.eff_fb[n_joints-1];
	curr.pos_fb[n_joints] = curr.pos_fb[n_joints-1];
	curr.vel_fb[n_joints] = curr.vel_fb[n_joints-1];
	
	acc_msg.data = curr.acc;
	acc_pub.publish(acc_msg);
}

void RobotHardwareInterface::write(ros::Duration elapsed_time) {
	if(!all_motors_sim && motors_en){
		// Write to driver.
		
		// -pi/2=2.5%=25 +pi/2=12.5%=125
		for(int id = 0; id < n_joints; id++){
			tmp_duty[id] = (curr.pos_cmd[id] + M_PI_2)*(100/M_PI) + 25;
		}
		
		seek();
		
		const int s = sizeof(tmp_duty[0])*n_joints; // Symetrical not writen.
		int r = ::write(drv_fd, (char*)tmp_duty.data(), s);
		if(r != s){
			ROS_WARN("write went wrong!");
		}
	}
}

void RobotHardwareInterface::motors_en_cb(
	const std_msgs::Bool::ConstPtr& msg
) {
	motors_en = msg->data;
	ROS_INFO(
		"Motors %s", motors_en ? "enabled" : "disabled"
	);
}


#include <srdfdom/model.h>
#include <srdfdom/srdf_writer.h>
#include <urdf_parser/urdf_parser.h>

struct ScopedLocale {
	ScopedLocale(const char* name = "C") {
		backup_ = setlocale(LC_ALL, nullptr);  // store current locale
		setlocale(LC_ALL, name);
	}
	~ScopedLocale() {
		setlocale(LC_ALL, backup_.c_str());  // restore locale
	}
	std::string backup_;
};


static shared_ptr<srdf::Model> readSRDF() {
	bool all_ok = true;
	bool ok;
	
	string urdf_xml_string;
	ok = ros::param::get("/robot_description", urdf_xml_string);
	if(!ok){
		all_ok = false;
		ROS_WARN("Cannot read \"/robot_description\" param!");
	}
	
	string srdf_xml_string;
	ok = ros::param::get("/robot_description_semantic", srdf_xml_string);
	if(!ok){
		all_ok = false;
		ROS_WARN(
			"Cannot read \"/robot_description_semantic\" param! "\
				"Maybe demo.launch is not started."
		);
	}
	
	if(!all_ok){
		ROS_WARN("Cannot read SRDF!");
		return nullptr;
	}


	ScopedLocale l("C");
	
	urdf::ModelInterfaceSharedPtr u = urdf::parseURDF(urdf_xml_string);
	shared_ptr<srdf::Model> s = std::make_shared<srdf::Model>();
	s->initString(*u, srdf_xml_string);

	return s;
}


void RobotHardwareInterface::readJointNames() {
	shared_ptr<srdf::Model> s = readSRDF();
	if(!s){
		ROS_ERROR("Cannot read joint names!");
	}
	
	for(auto& g: s->getGroups()){
		for(int i = 0; i < g.joints_.size(); i++){
			auto jn = g.joints_[i];
			if(jn != "virtual_joint"){
				joint_names.push_back(jn);
			}
		}
	}
	//joint_names.pop_back(); // Remove symetrical finger.
	
	n_joints = joint_names.size()-1;
	if(n_joints == 0){
		ROS_ERROR("Cannot read joint names!");
	}
}
