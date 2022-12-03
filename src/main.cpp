#include "main.h"
#include "pros/adi.h"
#include "pros/api_legacy.h"
#include "pros/llemu.hpp"
#include "pros/vision.hpp"
#include "pros/vision.h"
#include "pros/optical.h"
#include "pros/optical.hpp"

/*
 * Presence of these two variables here replaces _pros_ld_timestamp step in common.mk.
 * THis way we get equivalent behavior without extra .c file to compile, and this faster build.
 * Pros uses value from _PROS_COMPILE_TIMESTAMP to show time on cortex / remote (I think).
 * I'm not sure how _PROS_COMPILE_DIRECTORY is used, but it's not full path (only first 23 characters)
 * and not clear if it matters at all (likely also shows somewhere on cortex, which has no usage).
 * If both of these variables are not present, final binary output becomes larger. Not sure why.
 */
extern "C" char const *const _PROS_COMPILE_TIMESTAMP = __DATE__ " " __TIME__;
extern "C" char const *const _PROS_COMPILE_DIRECTORY = "";

#define max(a, b) ((a) < (b) ? (b) : (a))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define sign(x) ((x) > 0 ? 1 : -1)

int autonSide = 1;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{

	pros::lcd::initialize();

	if (autonSide == 1)
	{
		pros::lcd::set_text(1, "Selected Auton is Left");
	}
	if (autonSide == 2)
	{
		pros::lcd::set_text(1, "Selected Auton is Right");
	}
	if (autonSide == 3)
	{
		pros::lcd::set_text(1, "Selected Auton is Skills Left");
	}
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

class Autonomous
{

	pros::Controller master{CONTROLLER_MASTER};

	pros::Motor left_front{LeftFrontPort};
	pros::Motor left_middle{LeftMiddlePort, true};
	pros::Motor left_back{LeftBackPort};

	pros::Motor right_front{RightFrontPort, true};
	pros::Motor right_middle{RightMiddlePort};
	pros::Motor right_back{RightBackPort};

	pros::Motor FlyWheel1{fly_wheel1, MOTOR_GEARSET_36, true}; // Pick correct gearset (36 is red)
	pros::Motor Intake{intake, MOTOR_GEARSET_36, true};		   // Pick correct gearset (36 is red)
	pros::Vision vision_sensor{VisionPort, pros::E_VISION_ZERO_CENTER};

	int getLeftPos()
	{
		return (left_front.get_position() + left_middle.get_position() + left_back.get_position()) / 3;
	}

	int getRightPos()
	{
		return (right_front.get_position() + right_middle.get_position() + right_back.get_position()) / 3;
	}

	int getPos()
	{
		return (getLeftPos() + getRightPos()) / 2;
	}

	int getFlywheelPos()
	{
		return (FlyWheel1.get_position() + Intake.get_position()) / 2;
	}
	void Move(int ticks, int Lspeed, int Rspeed, bool intakeON, int intakeTicks, int intakeSpeed)
	{
		int startPos = getPos();
		int flywheelStartPos = getLeftPos();
		left_front.move(Lspeed * 127 / 200);
		left_middle.move(Lspeed * 127 / 200);
		left_back.move(Lspeed * 127 / 200);
		right_front.move(Rspeed * 127 / 200);
		right_middle.move(Rspeed * 127 / 200);
		right_back.move(-Rspeed * 127 / 200);
		if (intakeON)
		{
			FlyWheel1.move(intakeSpeed);
			Intake.move(intakeSpeed);
		}
		while (abs(getPos() - startPos) < ticks)
		{
			if (abs(getFlywheelPos() - flywheelStartPos) == intakeTicks && intakeON)
			{
				Intake.move(0);
				FlyWheel1.move(0);
			}
			pros::c::delay(10);
		}
		left_front.move(0);
		left_middle.move(0);
		left_back.move(0);
		right_front.move(0);
		right_middle.move(0);
		right_back.move(0);
		if (intakeON)
		{
			FlyWheel1.move(0);
			Intake.move(0);
		}
		pros::c::delay(100);
	}

	void flyWheelMove(int FlyWheelTicks, int FSpeed)
	{
		FlyWheel1.move(FSpeed);
		int flyWheelStartPos = getFlywheelPos();
		FlyWheel1.move(FSpeed);
		Intake.move(FSpeed);
		while (abs(getFlywheelPos() - flyWheelStartPos) < FlyWheelTicks)
		{
			pros::c::delay(10);
		}
		FlyWheel1.move(0);
		Intake.move(0);
		pros::c::delay(100);
	}

	void Turn(double degrees, int speed)
	{
		left_front.move_relative((degrees / 360) * 3525, speed);
		left_middle.move_relative((degrees / 360) * 3525, speed);
		left_back.move_relative((degrees / 360) * 3525, speed);

		right_front.move_relative((degrees / 360) * -3525, speed);
		right_middle.move_relative((degrees / 360) * -3525, speed);
		right_back.move_relative((degrees / 360) * 3525, speed);
	}
	void MoveVisionAssisted(int ticks, int speed)
	{
		int startPos = getPos();
		int timer = 350;

		while (abs(getPos() - startPos) < ticks && timer > 0)
		{
			int Lspeed = speed;
			int Rspeed = speed;

			if (vision_sensor.get_object_count() > 0)
			{
				pros::vision_object_s_t obj;

				if (vision_sensor.read_by_size(0, 1, &obj) == 1 && obj.top_coord + obj.height > 0)
				{
					// Positive offset means goal is left due to sensor being mounted upside down
					float offset = obj.x_middle_coord * sign(speed);
					printf("%d  %d\n", obj.width, obj.top_coord + obj.height);
					if (abs(offset) > 0.05)
					{
						if (offset < 0)
						{
							Rspeed = speed * (1 + max(offset, -100) * 0.005);
							// Rspeed = -50;
							// Lspeed = 50;
						}
						else
						{
							Lspeed = speed * (1 - min(offset, 100) * 0.005);
							// Rspeed = 50;
							// Lspeed = -50;
						}
					}
					if (obj.width > 280)
					{ // checks if the object is too close
						break;
					}
				}
			}

			left_front.move(Lspeed * 127 / 200);
			left_middle.move(Lspeed * 127 / 200);
			left_back.move(Lspeed * 127 / 200);
			right_front.move(Rspeed * 127 / 200);
			right_middle.move(Rspeed * 127 / 200);
			right_back.move(-Rspeed * 127 / 200);
			left_front.move(0);
			left_middle.move(0);
			left_back.move(0);
			right_front.move(0);
			right_middle.move(0);
			right_back.move(0);
		}
	}

public:
	void run()
	{
		pros::vision_signature_s_t sig1 = pros::c::vision_signature_from_utility(1, -2123, -1397, -1760, 8387, 10923, 9654, 3.100, 0);
		pros::vision_signature_s_t sig2 = pros::c::vision_signature_from_utility(2, 8257, 10627, 9442, -863, -373, -618, 2.000, 0);
		vision_sensor.set_signature(1, &sig1);
		vision_sensor.set_signature(2, &sig2);
		pros::c::adi_pin_mode(ShootPort, OUTPUT);
		pros::c::adi_digital_write(ShootPort, LOW); // write LOW to port 1 (solenoid may be extended or not, depending on wiring)
		/*pros::c::adi_pin_mode(expansionPort, OUTPUT);
	pros::c::adi_digital_write(expansionPort, LOW);*/

		if (autonSide == 1)
		{
			pros::c::adi_pin_mode(ShootPort, OUTPUT);
			pros::c::adi_digital_write(ShootPort, LOW);
			FlyWheel1.move_velocity(90);
			Intake.move_velocity(90);
			pros::c::delay(250);
			Move(175, -70, -70, false, 0, 0);
			pros::c::delay(50);
			Move(100, 100, 100, false, 0, 0);
			pros::c::delay(50);
			Turn(-9, 100);
			pros::c::delay(200);
			FlyWheel1.move_velocity(-87);
			Intake.move_velocity(-87);
			pros::c::delay(3000);
			pros::c::adi_digital_write(ShootPort, HIGH);
			pros::c::delay(400);
			pros::c::adi_digital_write(ShootPort, LOW);
			pros::c::delay(3000);
			FlyWheel1.move_velocity(-94);
			Intake.move_velocity(-94);
			pros::c::adi_digital_write(ShootPort, HIGH);
			pros::c::delay(400);
			pros::c::adi_digital_write(ShootPort, LOW);
			pros::c::delay(400);
			FlyWheel1.move_velocity(0);
			Intake.move_velocity(0);
		}
		else if (autonSide == 2)
		{
			pros::c::adi_pin_mode(ShootPort, OUTPUT);
			pros::c::adi_digital_write(ShootPort, LOW);
			Move(500, 100, 100, false, 0, 0);
			pros::c::delay(250);
			Turn(-90, 100);
			pros::c::delay(250);
			FlyWheel1.move_velocity(90);
			Intake.move_velocity(90);
			pros::c::delay(250);
			Move(175, -70, -70, false, 0, 0);
			pros::c::delay(50);
			Move(100, 100, 100, false, 0, 0);
			pros::c::delay(50);
			Turn(10, 100);
			pros::c::delay(200);
			FlyWheel1.move_velocity(-87);
			Intake.move_velocity(-87);
			pros::c::delay(4000);
			pros::c::adi_digital_write(ShootPort, HIGH);
			pros::c::delay(500);
			pros::c::adi_digital_write(ShootPort, LOW);
			pros::c::delay(4000);
			FlyWheel1.move_velocity(-93);
			Intake.move_velocity(-93);
			pros::c::adi_digital_write(ShootPort, HIGH);
			pros::c::delay(500);
			pros::c::adi_digital_write(ShootPort, LOW);
			pros::c::delay(400);
			FlyWheel1.move_velocity(0);
			Intake.move_velocity(0);
		}
		else if (autonSide == 3) // skills auto
		{
			pros::c::adi_pin_mode(ShootPort, OUTPUT);
			pros::c::adi_digital_write(ShootPort, LOW);
			FlyWheel1.move_velocity(90);
			Intake.move_velocity(90);
			pros::c::delay(250);
			Move(250, -70, -70, false, 0, 0);
			pros::c::delay(50);
			Move(100, 100, 100, false, 0, 0);
			pros::c::delay(50);
			Turn(-9, 100);
			pros::c::delay(200);
			FlyWheel1.move_velocity(-87);
			Intake.move_velocity(-87);
			pros::c::delay(3000);
			pros::c::adi_digital_write(ShootPort, HIGH);
			pros::c::delay(400);
			pros::c::adi_digital_write(ShootPort, LOW);
			pros::c::delay(3000);
			FlyWheel1.move_velocity(-94);
			Intake.move_velocity(-94);
			pros::c::adi_digital_write(ShootPort, HIGH);
			pros::c::delay(400);
			pros::c::adi_digital_write(ShootPort, LOW);
			pros::c::delay(400);
			FlyWheel1.move_velocity(0);
			Intake.move_velocity(0);
		}
	}
};

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
	Autonomous self_drive;
	self_drive.run();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol()
{
	pros::Controller master(CONTROLLER_MASTER);
	pros::Optical optical_sensor(opticalPort);
	pros::c::optical_rgb_s_t rgb_value;

	pros::Motor left_front(LeftFrontPort);
	pros::Motor left_middle(LeftMiddlePort, true);
	pros::Motor left_back(LeftBackPort);

	pros::Motor right_front(RightFrontPort, true);
	pros::Motor right_middle(RightMiddlePort);
	pros::Motor right_back(RightBackPort);

	pros::Motor FlyWheel1(fly_wheel1, MOTOR_GEARSET_36, true); // Pick correct gearset (36 is red)
	pros::Motor Intake(intake, MOTOR_GEARSET_36, true);		   // Pick correct gearset (36 is red)

	pros::c::adi_pin_mode(ShootPort, OUTPUT);
	pros::c::adi_digital_write(ShootPort, LOW); // write LOW to port 1 (solenoid may be extended or not, depending on wiring)
	/*pros::c::adi_pin_mode(expansionPort, OUTPUT);
	pros::c::adi_digital_write(expansionPort, LOW);*/
	int dead_Zone = 10; // the deadzone for the joysticks
	int defaultFlyWheelSpeed = -65;
	int FlyWheelSpeed = defaultFlyWheelSpeed;
	int FlyWheelOn = 0;

	while (true)
	{
		if (pros::lcd::read_buttons() == 4)
		{
			autonSide = 1;
		}
		else if (pros::lcd::read_buttons() == 2)
		{
			autonSide = 2;
		}
		else if (pros::lcd::read_buttons() == 6)
		{
			autonSide = 3;
		}
		if (autonSide == 1)
		{
			pros::lcd::set_text(1, "Selected Auton is Left");
		}
		if (autonSide == 2)
		{
			pros::lcd::set_text(1, "Selected Auton is Right");
		}
		if (autonSide == 3)
		{
			pros::lcd::set_text(1, "Selected Auton is Skills Left");
		}

		int leftSpeed = 0;
		int rightSpeed = 0;
		int analogY = master.get_analog(ANALOG_LEFT_Y);
		int analogX = master.get_analog(ANALOG_RIGHT_X);

		if (analogY == 0 && abs(analogX) > dead_Zone)
		{
			leftSpeed = analogX;
			rightSpeed = -analogX;
		}
		else if (analogX >= dead_Zone && analogY > dead_Zone)
		{
			leftSpeed = analogY;
			rightSpeed = analogY - analogX;
		}
		else if (analogX < -dead_Zone && analogY > dead_Zone)
		{
			leftSpeed = analogY + analogX;
			rightSpeed = analogY;
		}
		else if (analogX >= dead_Zone && analogY < -dead_Zone)
		{
			leftSpeed = analogY;
			rightSpeed = analogY + analogX;
		}
		else if (analogX < -dead_Zone && analogY < -dead_Zone)
		{
			leftSpeed = analogY - analogX;
			rightSpeed = analogY;
		}
		else if (analogX == 0 && abs(analogY) > dead_Zone)
		{
			leftSpeed = analogY;
			rightSpeed = analogY;
		}

		rgb_value = optical_sensor.get_rgb();
		// if (master.get_digital(DIGITAL_R1) && !rgb_value.blue && !rgb_value.blue)

		if (master.get_digital_new_press(DIGITAL_A))
		{
			FlyWheelSpeed = defaultFlyWheelSpeed;
		}

		if (master.get_digital_new_press(DIGITAL_Y))
		{
			FlyWheelSpeed = -defaultFlyWheelSpeed;
		}

		// X press changes flywheel speed to high (defualt setting)
		if (master.get_digital_new_press(DIGITAL_B))
		{
			FlyWheelSpeed = 0;
			FlyWheel1.move_velocity(FlyWheelSpeed);
			Intake.move_velocity(FlyWheelSpeed);
		}

		// B press changes flywheel speed to low setting
		if (master.get_digital_new_press(DIGITAL_X))
		{
			FlyWheelSpeed = -100;
		}
		FlyWheel1.move_velocity(FlyWheelSpeed);
		Intake.move_velocity(FlyWheelSpeed);

		if (master.get_digital_new_press(DIGITAL_R2))
		{
			pros::c::adi_digital_write(ShootPort, HIGH);
			pros::c::delay(250);
		}
		if (master.get_digital_new_press(DIGITAL_R2) == false)
		{
			pros::c::adi_digital_write(ShootPort, LOW);
		}
		if (master.get_digital_new_press(DIGITAL_DOWN))
		{
			//release end game
		}

		if (abs(leftSpeed) < 40 && abs(rightSpeed) < 40)
		{
			left_front.move(leftSpeed);
			left_middle.move(leftSpeed);
			left_back.move(leftSpeed);
			right_front.move(rightSpeed);
			right_middle.move(rightSpeed);
			right_back.move(-rightSpeed);
		}
		else
		{
			left_front.move(leftSpeed * 1.574);
			left_middle.move(leftSpeed * 1.574);
			left_back.move(leftSpeed * 1.574);
			right_front.move(rightSpeed * 1.574);
			right_middle.move(rightSpeed * 1.574);
			right_back.move(rightSpeed * -1.574);
		}

		pros::delay(10);
	}
}
