#include "main.h"
#include "pros/adi.h"
#include "pros/llemu.hpp"

// #define VISION_ENABLED
#ifdef VISION_ENABLED
#include "pros/vision.hpp"
#endif

// #define OPTICAL_ENABLED
#ifdef OPTICAL_ENABLED
#include "pros/optical.hpp"
#endif

// Disables all the logging. Comment it out and uncomment next line to get logging working.
#define log(...) (void)0
// #define log printf

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

enum AutonMode
{
	AutonLeft = 1,
	AutonRight = 2,
	AutonSkills = 3,
	AutonNone = 0,
};

AutonMode autonSide = AutonLeft;


void printAutonMessage()
{
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
	if (autonSide == AutonNone)
	{
		pros::lcd::set_text(1, "Selected Auton is None");
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
	pros::Motor right_back{RightBackPort, true};

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
	Program()
	{
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
		pros::c::adi_pin_mode(trajector, OUTPUT);
		pros::c::adi_digital_write(trajector, LOW);
	}

public:

	bool autonCompleted = false;

	void ShootDisk()
	{
		pros::c::adi_digital_write(ShootPort, HIGH);
		pros::c::delay(100);

		pros::c::adi_digital_write(ShootPort, LOW);
		pros::c::delay(100);
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

	void SetFlywheelVoltage(unsigned int voltage)
	{
		FlyWheel1.move_voltage(-voltage);
		Intake.move_voltage(-voltage);
	}

	void ShootDiskAccurate_old(unsigned int speed, int delay)
	{
		SetFlywheelVelocity(speed);
		pros::c::delay(delay);

		for (int i = 0; i < 200; i++)
		{
			auto vel = getFlywheelVelocity();

			log("%.1f\n", vel);
			pros::c::delay(10);
		}
		ShootDisk();
	}

	void ShootDiskAccurate_voltage(unsigned int voltage, int delay)
	{
		SetFlywheelVoltage(voltage);
		pros::c::delay(delay);

		ShootDisk();
	}

	void ShootDiskAccurate(int voltage)
	{
		/*Set RPM to max to speed up the motor as fast as possible
		FlyWheel1.move_velocity(-600);
		Intake.move_velocity(-600);
		while(c)*/
	}

	void SetDriveRelative(int ticks, int Lspeed, int Rspeed)
	{

		left_front.move_relative(ticks, Lspeed);
		left_middle.move_relative(ticks, Lspeed);
		left_back.move_relative(ticks, Lspeed);

		right_front.move_relative(ticks, Rspeed);
		right_middle.move_relative(ticks, Rspeed);
		right_back.move_relative(ticks, Rspeed);
	}

	void SetDrive(int Lspeed, int Rspeed)
	{

		left_front.move(Lspeed);
		left_middle.move(Lspeed);
		left_back.move(Lspeed);
		right_front.move(Rspeed);
		right_middle.move(Rspeed);
		right_back.move(Rspeed);
	}
};
/*
float Kp = 0.2;
float Kd = 0.0;
float Ki = 0.0;
int targetVelocity = 0;

int shootPID() {
	while() {
		lastError = currentVelocity;

		float error = targetVelocity - getFlywheelVelocity()();
		float deltaT = ;

		float P = error * Kp

		float I += error * deltaT * Ki

		float D = ((currentVelocity - lastVelocity) / deltaT) * Kd;

		int output = P + I - D;


	}
}

int drivePID() {
	while() {

	}
}*/

/**
 *
 * Autonomous - Autonomous related code
 *
 */
class Autonomous : public Program
{

private:
	double fullCircleTicks = (2130);

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

	double getAngle()
	{
		return ((((getLeftPos() - getRightPos()) / 2.0) / fullCircleTicks) * 360.0);
	}

	void Move(int ticks, int Lspeed, int Rspeed, int timeOut)
	{
		int counter = 0;
		int startPos = getPos();

		SetDriveRelative(ticks, Rspeed * 127 / 200, Lspeed * 127 / 200);

		while (abs(getPos() - startPos) < abs(ticks) && counter <= timeOut)
		{
			pros::c::delay(10);
			counter = counter + 10;
		}

		SetDrive(0, 0);
		pros::c::delay(100);
	}

	void Turn(double degrees, int speed, int timeOut)
	{
		int counter = 0;
		/*int startLeftPos = getLeftPos();
		int startRightPos = getRightPos();*/
		int ticks = ((degrees / 360) * fullCircleTicks);
		int startAngle = getAngle();

		printf("Turn degrees = %f S = %d StartAngle = %f \n", degrees, speed, startAngle);

		left_front.move_relative(ticks * 1.005, speed);
		left_middle.move_relative(ticks * 1.005, speed);
		left_back.move_relative(ticks * 1.005, speed);

		right_front.move_relative(-ticks * 1.005, speed);
		right_middle.move_relative(-ticks * 1.005, speed);
		right_back.move_relative(-ticks * 1.005, speed);

		// while (abs(getLeftPos() - startLeftPos) < ticks - 100 && abs(getRightPos() - startRightPos) < ticks - 100)
		while (abs(getAngle() - startAngle) < abs(degrees) && counter < timeOut)
		{
			printf("   Degrees Turned = %f \n", (getAngle() - startAngle));
			pros::c::delay(10);
			counter = counter + 10;
		}

		printf(" Degrees Turned at End = %f \n", (getAngle() - startAngle));
		SetDrive(0, 0);
		pros::c::delay(100);
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
	void runLeft()
	{
		// prep flywheel
		SetFlywheelVoltage(12000);
		pros::c::delay(400);

		// Turn to aim at goal
		Turn(-15, 50, 500);

		// Shoot the two preloads
		ShootDiskAccurate_voltage(11500, 2500);

		ShootDiskAccurate_voltage(11500, 1250);

		// Start Roller
		SetRollerVelocity(90);

		// Turn back to start
		Turn(15, 50, 500);

		// Move back towards roller
		Move(-175, 70, 70, 350);
		pros::c::delay(50);

		// Move forwards from roller after it's turned
		Move(100, 100, 100, 1000);

		// prep flywheel
		SetFlywheelVoltage(12000);

		// Turn towards stack of discs
		Turn(32, 70, 1000);

		// Pick up discs
		Move(1700, 90, 90, 5000);
		pros::c::delay(200);

		// Turn towards goal
		Turn(-55, 70, 5000);

		// Shoot three discs
		ShootDiskAccurate_voltage(10500, 1000);

		ShootDiskAccurate_voltage(10500, 1500);

		ShootDiskAccurate_voltage(10500, 1000);
	}

	void runRight()
	{
		// prep flywheel
		SetFlywheelVoltage(12000);
		pros::c::delay(400);

		Move(950, 100, 100, 3000);
		pros::c::delay(50);

		Turn(23, 100, 5000);

		ShootDiskAccurate_voltage(11000, 2000);

		ShootDiskAccurate_voltage(11000, 1500);

		ShootDiskAccurate_voltage(11000, 1500);

		// prep for future shots & disk pick up
		SetFlywheelVoltage(9100);
		pros::c::delay(500);

		// turn towards 2 disks
		Turn(-58.5, 75, 5000);

		// pick up 2 disks
		Move(1400, 100, 100, 3000);

		// Move backwards toward roller
		Move(-2800, 120, 120, 3000);

		// turn towards roller
		Turn(44, 100, 5000);

		// turn roller
		SetRollerVelocity(90);
		Move(-160, 100, 100, 1000);
		pros::c::delay(500);

		// move out, prep to shoot
		Move(100, 100, 100, 400);

		/*
		ShootDiskAccurate_voltage(9100, 1000);
		ShootDiskAccurate_voltage(9100, 1000);*/
	}

	void runSkills()
	{
		// Roller 1
		SetRollerVelocity(90);

		Move(-215, 35, 35, 2000);
		pros::c::delay(200);

		Move(100, 100, 100, 1000);

		SetFlywheelVoltage(8000);

		Move(550, 100, 100, 1000);
		pros::c::delay(2000);

		// Turn
		Turn(90, 35, 5000);
		pros::c::delay(500);

		Move(-885, 70, 70, 5000);

		// Roller 2
		SetRollerVelocity(90);

		Move(-215, 70, 70, 1000);
		pros::c::delay(200);

		Move(215, 100, 100, 1000);
		pros::c::delay(50);

		// Turn and Move Across Field
		Turn(-45, 35, 5000);
		Move(5000, 100, 100, 20000);

		/*Expansion
		pros::c::adi_digital_write(expansionPort2, HIGH);
		pros::c::adi_digital_write(expansionPort, HIGH);*/
	}

	void run()
	{
		autonCompleted = true;
		
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
class OpControl : public Program
{
public:
	void opcontrol()
	{
		pros::Controller master(CONTROLLER_MASTER);
#ifdef OPTICAL_ENABLED
		pros::Optical optical_sensor(opticalPort);
#endif

		int dead_Zone = 10; // the dead zone for the joysticks
		const int defaultFlyWheelVoltage = -8900;
		int FlyWheelVoltage = defaultFlyWheelVoltage;
		int FlyWheelOn = 0;

		if (autonCompleted) {
			pros::c::adi_digital_write(trajector, HIGH);
		}

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
				// Flywheel is on low setting
				if (master.get_digital_new_press(DIGITAL_A))
				{
					FlyWheelVoltage = defaultFlyWheelVoltage;
				}

			// Flywheel is powered, reverse
			if (master.get_digital_new_press(DIGITAL_Y))
			{
				FlyWheelVoltage = -defaultFlyWheelVoltage;
			}

			// Flywheel is stopped
			if (master.get_digital_new_press(DIGITAL_B))
			{
				FlyWheelVoltage = 0;
			}

			// Flywheel speed is high
			if (master.get_digital_new_press(DIGITAL_X))
			{
				FlyWheelVoltage = -9300;
			}

			FlyWheel1.move_voltage(FlyWheelVoltage);
			Intake.move_voltage(FlyWheelVoltage);

			if (master.get_digital_new_press(DIGITAL_R2))
			{
				FlyWheel1.move_voltage(-12000);
				Intake.move_voltage(-12000);

				for (int i = 0; i < 3 && master.get_digital(DIGITAL_R2) && !master.get_digital(DIGITAL_L1) && !master.get_digital(DIGITAL_R1); i++)
				{
					FlyWheel1.move_voltage(FlyWheelVoltage);
					Intake.move_voltage(FlyWheelVoltage);
					ShootDisk();
					if (FlyWheelVoltage == defaultFlyWheelVoltage)
				{
					pros::c::delay(550);
				}
				else
				{
					pros::c::delay(700);
				}
				}
			}

			//override for trajector
			if (master.get_digital(DIGITAL_DOWN))
			{
				pros::c::adi_digital_write(trajector, HIGH);
			}

			if (master.get_digital(DIGITAL_UP))
			{
				pros::c::adi_digital_write(trajector, LOW);
			}

			/**
			 * End-game expansion
			 */
			if (master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_R1))
			{
				pros::c::adi_digital_write(expansionPort, HIGH);
				pros::c::adi_digital_write(expansionPort2, HIGH);
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
