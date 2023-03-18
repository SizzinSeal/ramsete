#include "main.h"
#include "lemlib/api.hpp"


// controllers
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain motors
pros::Motor leftFront(-3, pros::E_MOTOR_GEARSET_06);
pros::Motor leftMiddle(-14, pros::E_MOTOR_GEARSET_06);
pros::Motor leftBack(-12, pros::E_MOTOR_GEARSET_06);
pros::Motor rightFront(19, pros::E_MOTOR_GEARSET_06);
pros::Motor rightMiddle(20, pros::E_MOTOR_GEARSET_06);
pros::Motor rightBack(1, pros::E_MOTOR_GEARSET_06);

// drivetrain motor groups
pros::MotorGroup leftDrive({leftFront, leftMiddle, leftBack});
pros::MotorGroup rightDrive({rightFront, rightMiddle, rightBack});

// sensors
pros::Imu imu(6);

// odom sensors
lemlib::OdomSensors_t sensors {
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&imu
};

// lateral PD controller
lemlib::ChassisController_t lateralController {
	8,
	30,
	1,
	100,
	3,
	500,
	5
};

// angular PD controller
lemlib::ChassisController_t angularController {
	4,
	40,
	1,
	100,
	3,
	500,
	0
};

// drivetrain
lemlib::Drivetrain_t drivetrain {
	&leftDrive,
	&rightDrive,
	10.2,
	3.25,
	360
};

// chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);
