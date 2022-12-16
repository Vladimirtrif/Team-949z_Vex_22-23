#include "main.h"
#include "pros/adi.h"
#include "pros/llemu.hpp"

// #define VISION_ENABLED
#ifdef VISION_ENABLED
#  include "pros/vision.hpp"
#endif

// #define OPTICAL_ENABLED
#ifdef OPTICAL_ENABLED
#  include "pros/optical.hpp"
#endif

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

enum AutonMode {
	AutonLeft = 1,
	AutonRight = 2,
	AutonSkills = 3,
};

AutonMode autonSide = AutonRight;

void printAutonMessage() {
	if (autonSide == AutonLeft)
	{
		pros::lcd::set_text(1, "Selected Auton is Left");
	}
	if (autonSide == AutonRight)
	{
		pros::lcd::set_text(1, "Selected Auton is Right");
	}
	if (autonSide == AutonSkills)
	{
		pros::lcd::set_text(1, "Selected Auton is Skills");
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	printAutonMessage();
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


/**
 * 
 * Program - common code between Autonomous & manual driving
 * 
*/
class Program
{
protected:
	pros::Controller master{CONTROLLER_MASTER};

	pros::Motor left_front{LeftFrontPort};
	pros::Motor left_middle{LeftMiddlePort, true};
	pros::Motor left_back{LeftBackPort};

	pros::Motor right_front{RightFrontPort, true};
	pros::Motor right_middle{RightMiddlePort};
	pros::Motor right_back{RightBackPort};

	// Should be E_MOTOR_GEARSET_06 - 600 rpm
	pros::Motor FlyWheel1{fly_wheel1, MOTOR_GEARSET_36, true}; // Pick correct gearset (36 is red)
	pros::Motor Intake{intake, MOTOR_GEARSET_36, true};		   // Pick correct gearset (36 is red)
#ifdef VISION_ENABLED
	pros::Vision vision_sensor{VisionPort, pros::E_VISION_ZERO_CENTER};
#endif

#ifdef OPTICAL_ENABLED
	pros::vision_signature_s_t sig1 = pros::c::vision_signature_from_utility(1, -2123, -1397, -1760, 8387, 10923, 9654, 3.100, 0);
	pros::vision_signature_s_t sig2 = pros::c::vision_signature_from_utility(2, 8257, 10627, 9442, -863, -373, -618, 2.000, 0);
#endif

	// constructor
public:
	Program() {
#ifdef OPTICAL_ENABLED
		vision_sensor.set_signature(1, &sig1);
		vision_sensor.set_signature(2, &sig2);
#endif
		pros::c::adi_pin_mode(ShootPort, OUTPUT);
		pros::c::adi_digital_write(ShootPort, LOW);
		pros::c::adi_pin_mode(expansionPort, OUTPUT);
		pros::c::adi_digital_write(expansionPort, LOW);
		pros::c::adi_pin_mode(expansionPort2, OUTPUT);
		pros::c::adi_digital_write(expansionPort2, LOW);
	}

public:
	void ShootDisk()
	{
		pros::c::adi_digital_write(ShootPort, HIGH);
		pros::c::delay(250);

		pros::c::adi_digital_write(ShootPort, LOW);
		pros::c::delay(250);
	}

	double getFlywheelVelocity()
	{
		return -(FlyWheel1.get_actual_velocity() + Intake.get_actual_velocity()) / 2;
	}

	void SetRollerVelocity(unsigned int speed)
	{
		FlyWheel1.move_velocity(speed);
		Intake.move_velocity(speed);
	}

	void SetFlywheelVelocity(unsigned int speed)
	{
		FlyWheel1.move_velocity(-speed);
		Intake.move_velocity(-speed);
	}

	void ShootDiskAccurate(unsigned int speed)
	{
		SetFlywheelVelocity(speed);
		pros::c::delay(2000);

		for (int i = 0; i < 200; i++) {
			auto vel = getFlywheelVelocity();

			printf("%.1f\n", vel);
			pros::c::delay(10);
		}
		// ShootDisk();
	}

	void ShootDiskAccurate3(unsigned int speed)
	{
		SetFlywheelVelocity(speed);
		pros::c::delay(1000);

		auto prev = getFlywheelVelocity();
		for (int i = 0; i < 200; i++) {
			auto vel = getFlywheelVelocity();
			auto delta = vel - speed;
			// If we are pretty close to target, and we speed changes in the direction of the target
			// (i.e. if we are slower than target & speed is increasing over time), then stop and shoot.
			if (abs(delta) <= 1) {
				if (delta * (vel - prev) <= 0)
					break;
			}
			prev = vel;

			printf("%.1f\n", vel);
			pros::c::delay(10);
		}
		ShootDisk();
	}

	void ShootDiskAccurate1(unsigned int speed)
	{
		SetFlywheelVelocity(speed);
		pros::c::delay(1000);

		for (int i = 0; i < 200; i++) {
			auto vel = getFlywheelVelocity();

			// Further out we are - the more voltage we need to faster get there
			// But once we reach the speed, we need some power to simply maintain momentum
			int voltage = 180 * (speed - vel) + speed * 60;

			// Max is +-12,000
			if (voltage >= 12000)
				voltage = 12000;
			// We do not go backwards - we simply let time slow down the wheel!
			if (voltage < 0)
				voltage = 0;

			FlyWheel1.move_voltage(-voltage);
			Intake.move_voltage(-voltage);

			printf("%.1f %d\n", vel, voltage);
			pros::c::delay(10);
		}

		SetFlywheelVelocity(speed);
		ShootDisk();
	}

	void SetDrive(int Lspeed, int Rspeed)
	{
		left_front.move(Lspeed);
		left_middle.move(Lspeed);
		left_back.move(Lspeed);
		right_front.move(Rspeed);
		right_middle.move(Rspeed);
		right_back.move(-Rspeed);
	}
};


/**
 * 
 * Autonomous - Autonomous related code
 * 
*/
class Autonomous : public Program
{

private:
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

	void Move(int ticks, int Lspeed, int Rspeed, int timeOut)
	{
		int counter = 0;
		int startPos = getPos();

		SetDrive(Lspeed * 127 / 200, Rspeed * 127 / 200);

		while (abs(getPos() - startPos) < ticks && counter <= timeOut)
		{
			pros::c::delay(10);
			counter = counter + 10;
		}

		SetDrive(0, 0);
		pros::c::delay(100);
	}

	void Turn(double degrees, int speed)
	{
		left_front.move_relative((degrees / 360) * 3525 * 2/3, speed);
		left_middle.move_relative((degrees / 360) * 3525 * 2/3, speed);
		left_back.move_relative((degrees / 360) * 3525 * 2/3, speed);

		right_front.move_relative((degrees / 360) * -3525 * 2/3, speed);
		right_middle.move_relative((degrees / 360) * -3525 * 2/3, speed);
		right_back.move_relative((degrees / 360) * 3525 * 2/3, speed);
	}

#ifdef VISION_ENABLED
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

			SetDrive(Lspeed * 127 / 200, Rspeed * 127 / 200);
		}
		SetDrive(0, 0);
	}
#endif

public:
	void runLeft() {
		// prep flywheel
		SetFlywheelVelocity(84);

		Turn(-13.5, 100);
		pros::c::delay(200);

		ShootDiskAccurate(84);

		ShootDiskAccurate(84);

		SetRollerVelocity(90);

		Turn(13.5, 100);
		pros::c::delay(250);

		Move(140, -70, -70, 350);
		pros::c::delay(50);

		Move(100, 100, 100, 1000);
		pros::c::delay(50);
	}

	void runRight() {
		// prep flywheel
		SetFlywheelVelocity(83);

		Turn(24, 100);
		pros::c::delay(300);

		ShootDiskAccurate(83);

		ShootDiskAccurate(89);

		Turn(61, 100);
		pros::c::delay(1000);

		Move(470, 100, 100, 3000);
		pros::c::delay(500);

		Turn(-75, 100);
		pros::c::delay(450);

		SetRollerVelocity(90);
		pros::c::delay(250);

		Move(140, -70, -70, 400);
		pros::c::delay(50);

		Move(100, 100, 100, 10000);
		pros::c::delay(50);
	}

	void runSkills() {
		// prep flywheel
		SetFlywheelVelocity(87);

		Turn(-6, 100);
		pros::c::delay(200);

		ShootDiskAccurate(87);

		ShootDiskAccurate(87);

		Turn(6, 100);
		pros::c::delay(400);

		SetRollerVelocity(90);
		pros::c::delay(250);

		Move(175, -70, -70, 1000);
		pros::c::delay(200);

		Move(100, 100, 100, 1000);
		pros::c::delay(50);

		SetRollerVelocity(0);
		Turn(20, 100);
		pros::c::delay(750);

		pros::c::adi_digital_write(expansionPort2, HIGH);
		pros::c::adi_digital_write(expansionPort, HIGH);
	}

	void run()
	{
		if (autonSide == AutonLeft)
		{
			runLeft();
		}
		else if (autonSide == AutonRight)
		{
			runRight();
		}
		else if (autonSide == AutonSkills)
		{
			runSkills();
		}

		SetRollerVelocity(0);
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
class OpControl: public Program {
public:
	void opcontrol() {
		pros::Controller master(CONTROLLER_MASTER);
	#ifdef OPTICAL_ENABLED
		pros::Optical optical_sensor(opticalPort);
	#endif

		int dead_Zone = 10; // the dead zone for the joysticks
		const int defaultFlyWheelSpeed = -65;
		int FlyWheelSpeed = defaultFlyWheelSpeed;
		int FlyWheelOn = 0;

		ShootDiskAccurate(84);

		while (true)
		{
			/**
			 * Autonomous selection
			*/
			if (pros::lcd::read_buttons() == 4)
			{
				autonSide = AutonLeft;
			}
			else if (pros::lcd::read_buttons() == 2)
			{
				autonSide = AutonRight;
			}
			else if (pros::lcd::read_buttons() == 1)
			{
				autonSide = AutonSkills;
			}
			printAutonMessage();

			/**
			 * Drivetrain
			*/
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

			if (abs(leftSpeed) < 40 && abs(rightSpeed) < 40)
			{
				SetDrive(leftSpeed, rightSpeed);
			}
			else
			{
				SetDrive(leftSpeed * 1.574, rightSpeed * 1.574);
			}

	#ifdef OPTICAL_ENABLED
			auto rgb_value = optical_sensor.get_rgb();
			if (master.get_digital(DIGITAL_R1) && !rgb_value.blue && !rgb_value.blue)
	#endif 

			/**
			 * Flywheel
			*/
			// Roller, flywheel is not powered
			if (master.get_digital_new_press(DIGITAL_A))
			{
				FlyWheelSpeed = defaultFlyWheelSpeed;
			}

			// Flywheel is powered, low setting
			if (master.get_digital_new_press(DIGITAL_Y))
			{
				FlyWheelSpeed = -defaultFlyWheelSpeed;
			}

			if (master.get_digital_new_press(DIGITAL_B))
			{
				FlyWheelSpeed = 0;
			}

			// Flywheel speed is high
			if (master.get_digital_new_press(DIGITAL_X))
			{
				FlyWheelSpeed = -100;
			}

			FlyWheel1.move_velocity(FlyWheelSpeed);
			Intake.move_velocity(FlyWheelSpeed);

			/**
			 * Shooting disks
			*/
			if (master.get_digital_new_press(DIGITAL_R2))
			{
				ShootDisk();
			}

			/**
			 * End-game expansion
			*/
			if (master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_R1))
			{
				pros::c::adi_digital_write(expansionPort, HIGH);
				pros::c::adi_digital_write(expansionPort2, HIGH);
				pros::c::delay(500);
			}

			pros::delay(10);
		}
	}
};

void opcontrol()
{
	OpControl program;
	program.opcontrol();
}
