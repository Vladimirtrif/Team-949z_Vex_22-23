#include "main.h"
#include "pros/adi.h"
#include "pros/llemu.hpp"
#include "pros/vision.hpp"
#include "pros/vision.h"

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
#define abs(x) ((x) > 0 ? (x) : -(x))

enum AutonMode
{
	AutonLeft = 1,
	AutonRight = 2,
	AutonSkills = 3,
	AutonNone = 0,
};

enum Signatures
{
	// Has to start with 1!
	Yellow = 1,
	Blue = 2,
	Red = 3,
};

AutonMode autonSide = AutonRight;


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

	pros::Vision vision_sensor{VisionPort, pros::E_VISION_ZERO_CENTER};
	pros::vision_signature_s_t YELLOW_SIG = pros::c::vision_signature_from_utility(Signatures::Yellow, 1275, 1831, 1552, -3987, -3569, -3778, 8.800, 0);
	pros::vision_signature_s_t BLUE_SIG = pros::c::vision_signature_from_utility(Signatures::Blue, -2571, -1031, -1800, 7753, 9363, 8558, 5.000, 0);
	pros::vision_signature_s_t RED_SIG = pros::c::vision_signature_from_utility(Signatures::Red, 7365, 7925, 7646, -477, 89, -194, 6.200, 0);
	// constructor
public:
	Program()
	{
		vision_sensor.set_signature(Signatures::Yellow, &YELLOW_SIG);
		vision_sensor.set_signature(Signatures::Blue, &BLUE_SIG);
		vision_sensor.set_signature(Signatures::Red, &RED_SIG);
		vision_sensor.set_exposure(80);
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

	void SetRollerVelocity(unsigned int speed)
	{
		FlyWheel1.move_velocity(speed);
		Intake.move_velocity(speed);
	}

	void SetFlywheelVoltage(unsigned int voltage)
	{
		FlyWheel1.move_voltage(-voltage);
		Intake.move_voltage(-voltage);
	}


	void ShootDiskAccurate_voltage(unsigned int voltage, int delay)
	{
		SetFlywheelVoltage(voltage);
		pros::c::delay(delay);

		SetFlywheelVoltage(-12000);
		ShootDisk();

		SetFlywheelVoltage(voltage);
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

		SetDriveRelative(ticks, Lspeed * 127 / 200, Rspeed * 127 / 200);

		while (abs(getPos() - startPos) < abs(ticks) && counter <= timeOut)
		{
			pros::c::delay(10);
			counter = counter + 10;
		}

		SetDrive(0, 0);
		pros::c::delay(100);
	}

	void Move_Old(int ticks, int Lspeed, int Rspeed, int timeOut)
	{
		int counter = 0;
		int startPos = getPos();

		SetDrive(Lspeed * 127 / 200, Rspeed * 127 / 200);

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

		printf("Turn degrees = %.2f S = %d StartAngle = %d ticks = %d getLeftPos = %d getRightPos = %d Timeout = %d \n", degrees, speed, startAngle, ticks, getLeftPos(), getRightPos(), timeOut);

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

		printf(" abs Degrees Turned at End = %f degrees turned at end = %f counter = %d abs(degrees) = %.2f \n", abs(getAngle() - startAngle), (getAngle() - startAngle), counter, abs(degrees));
		SetDrive(0, 0);
		pros::c::delay(100);
	}

	void MoveVisionAssisted(int ticks, int speed, int timeOut)
	{
		int startPos = getPos();

		while (abs(getPos() - startPos) < ticks && timeOut > 0)
		{
			int Lspeed = speed;
			int Rspeed = speed;

			if (vision_sensor.get_object_count() > 0)
			{
				pros::vision_object_s_t obj;

				if (vision_sensor.read_by_size(0, 1, &obj) == 1 && obj.signature == Signatures::Yellow && obj.top_coord + obj.height > 0)
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
					if (obj.width > 150)
					{ // checks if the object is too close
						break;
					}
				}
			}

			SetDrive(Lspeed * 127 / 200, Rspeed * 127 / 200);
			pros::c::delay(5);
			timeOut = timeOut - 5;
		}
		SetDrive(0, 0);
	}

public:
	void runLeft()
	{
		// prep flywheel
		SetFlywheelVoltage(10500);
		pros::c::delay(750);

		// Turn to aim at goal
		Turn(-19, 50, 500);

		// Shoot the two preloads
		ShootDiskAccurate_voltage(10500, 1800);

		ShootDiskAccurate_voltage(10500, 1800);

		// Start Roller
		SetRollerVelocity(90);

		//Turn back to start
		Turn(19, 50, 500);

		// Move back towards roller
		Move(-175, 70, 70, 350);
		pros::c::delay(50);

		// Move forwards from roller after it's turned
		Move(100, 100, 100, 1000);

		// prep flywheel
		SetFlywheelVoltage(12000);

		// Turn towards stack of discs
		Turn(40, 70, 1000);

		// Pick up discs
		Move(1700, 90, 90, 5000);
		pros::c::delay(200);

		// Turn towards goal
		Turn(-59.4, 70, 5000);

		SetFlywheelVoltage(10175);
		pros::c::delay(500);

		// Shoot three discs
		ShootDiskAccurate_voltage(10175, 1000);

		ShootDiskAccurate_voltage(10175, 1200);

		ShootDiskAccurate_voltage(10175, 1200);
	}

	void runRight()
	{
		// prep flywheel
		SetFlywheelVoltage(10200);
		pros::c::delay(850);

		Move(950, 100, 100, 3000);
		pros::c::delay(50);

		Turn(24.5, 100, 1000);

		ShootDiskAccurate_voltage(10200, 1000);

		ShootDiskAccurate_voltage(10200, 1500);

		ShootDiskAccurate_voltage(10200, 1500);

		//prep for future shots & disk pick up
		SetFlywheelVoltage(9100);
		pros::c::delay(500);

		// turn towards 2 disks
		Turn(-65, 75, 2000);

		// pick up 2 disks
		Move(1400, 100, 100, 3000);

		// Move backwards toward roller
		Move_Old(2825, -120, -120, 3000);

		// turn towards roller
		Turn(42, 100, 2000);

		// turn roller
		SetRollerVelocity(90);
		Move(-320, 100, 100, 1000);
		pros::c::delay(350);

		SetFlywheelVoltage(11000);

		// move out, prep to shoot
		Move(100, 100, 100, 400);

		Turn(16, 60, 250);

		ShootDiskAccurate_voltage(11000, 500);
	}

	void runSkills()
	{
		//prep flywheel
		SetFlywheelVoltage(6700);
		pros::c::delay(400);

		//move towards goal
		Move(800, 60, 60, 3000);

		//shoot preloads 
		ShootDiskAccurate_voltage(6700, 2000);

		ShootDiskAccurate_voltage(6700, 2000);
		pros::c::delay(100);

		//move back
		Move(-800, 60, 60, 3000);

		//turn towards discs and pick them up
		Turn(-90, 50, 2000);
		pros::c::delay(200);

		SetFlywheelVoltage(10000);
		pros::c::delay(250);

		Move(1500, 50, 50, 8000);

		//wait for intake to finish loading discs
		SetFlywheelVoltage(10000);
		pros::c::delay(1000);

		//move back a bit and turn to shoot
		Move(-700, 60, 60, 3000);
		pros::c::delay(100);

		Turn(105, 50, 2000);
		pros::c::delay(100);

		//move towards goal
		Move(700, 60, 60, 3000);

		SetFlywheelVoltage(6700);
		pros::c::delay(400);

		//shoot the three discs
		ShootDiskAccurate_voltage(6700, 2000);

		ShootDiskAccurate_voltage(6700, 2000);

		ShootDiskAccurate_voltage(6700, 2000);
		pros::c::delay(100);
		
		//turn towards other pile of three
		Turn(-172, 50, 3000);

		//prep intake and move towards stack of three
		SetFlywheelVoltage(10000);
		pros::c::delay(250);
		Move(2700, 50, 50, 9000);

		//wait for it to pick up 
		SetFlywheelVoltage(10000);
		pros::c::delay(1500);

		//prep intake and move towards goal
		SetFlywheelVoltage(6700);
		pros::c::delay(250);
		Move(-2700, 50, 50, 8000);

		//turn to shoot
		Turn(172, 50, 3000);

		//shoot the three discs
		ShootDiskAccurate_voltage(6700, 2000);

		ShootDiskAccurate_voltage(6700, 2000);

		ShootDiskAccurate_voltage(6700, 2000);
		pros::c::delay(100);
		

		/*rollers
		 //Roller 1
		Turn(90, 50, 3000);
		Move(-1500, 35, 35, 1000);
		SetRollerVelocity(90);
		SetDrive(-30,-30);
		pros::c::delay(350);

		Move(100, 100, 100, 1000);

		SetFlywheelVoltage(8000);

		Move(550, 100, 100, 1000);
		pros::c::delay(1000);

		// Turn
		Turn(85, 35, 5000);
		pros::c::delay(500);

		// Roller 2
        SetFlywheelVoltage(0);
        Move(-1200, 70, 70, 2000);
        SetRollerVelocity(90);
        SetDrive(-30, -30);
        pros::c::delay(350);
		
        // Move away from roller 2
		Move(215, 100, 100, 1000);
		pros::c::delay(50);

		/*Expansion
		Turn(-30, 35, 5000);
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

		int dead_Zone = 10; // the dead zone for the joysticks
		const int defaultFlyWheelVoltage = -8810;
		const int highFlywheelVoltage = -9210;
		int FlyWheelVoltage = defaultFlyWheelVoltage;
		int FlyWheelOn = 0;
		bool trajectorDown = true;

		if (autonCompleted) {
			pros::c::adi_digital_write(trajector, LOW);
		}

		unsigned int autoTurning = 0;
		bool onTarget = false;

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

			if (master.get_digital(DIGITAL_L2)) {
				pros::vision_object_s_t obj;

				// If you ue read_by_size(), and you have yellow signature in the list, you may get yellow disks in basket
				// They might be uneven in bsket. It's better to specify specific signature.
				obj = vision_sensor.get_by_sig(0, Signatures::Blue);
				if (obj.signature != Signatures::Blue) {
					obj = vision_sensor.get_by_sig(0, Signatures::Red);
				}

				if (obj.signature == Signatures::Blue || obj.signature == Signatures::Red)
				{
					autoTurning++;
					// Positive offset means goal is on the left due to sensor being mounted upside down
					float offset = obj.x_middle_coord * 1.0 / obj.width;
					printf("offset = %.2f mid = %d, w = %d  top = %d\n", offset, obj.x_middle_coord, obj.width, obj.top_coord + obj.height);
					
					// we need to give it some push initially, but once roboto starts moving we need to reduce power
					// not to over shoot.
					if (abs(offset) < 0.1) {
						printf("\n--- on target ---\n\n");
						onTarget = true;
						autoTurning = 0;
						offset = 0;
					} else if (autoTurning > 25)
						offset = 30 * sign(offset);
					else if (abs(offset) > 0.5)
						offset = 40 * sign(offset);
					else
						offset = 37 * sign(offset);

					// It's harder to tuen with both sides having same power, as it's easy to overshoot.
					// At the same time putting power only on one set of wheels results in robot moving forward / backwards.
					// It's better to have most power on one side, and a bit of power on another side not to let robot move forward.
					rightSpeed = offset;
					leftSpeed  = -offset / 1.8;
				} else {
					printf("   miss %d %d\n", vision_sensor.get_object_count(), obj.signature);
				}
			} else {
				autoTurning = 0;
				onTarget = false;
			}

			if (abs(leftSpeed) < 40 && abs(rightSpeed) < 40)
			{
				SetDrive(leftSpeed, rightSpeed);
			}
			else
			{
				SetDrive(leftSpeed * 1.574, rightSpeed * 1.574);
			}

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
				FlyWheelVoltage = highFlywheelVoltage;
			}

			FlyWheel1.move_voltage(FlyWheelVoltage);
			Intake.move_voltage(FlyWheelVoltage);

			if (master.get_digital_new_press(DIGITAL_R2) || onTarget)
			{
				SetFlywheelVoltage(12000);

				for (
					int i = 0;
					i < 3 &&
						(master.get_digital(DIGITAL_R2) || onTarget && master.get_digital(DIGITAL_L2)) &&
						!master.get_digital(DIGITAL_L1) &&
						!master.get_digital(DIGITAL_R1);
					i++)
				{
					ShootDisk();
					if (FlyWheelVoltage == defaultFlyWheelVoltage)
					{
						pros::c::delay(200);
					}
					else
					{
						pros::c::delay(300);
					}
				}
			}

			//override for trajector
			if (master.get_digital_new_press(DIGITAL_DOWN))
			{
				trajectorDown = !trajectorDown;
				
			}

			if (trajectorDown == false) {
					pros::c::adi_digital_write(trajector, HIGH);
					//FlyWheelVoltage = .9 * FlyWheelVoltage;
				} else {
					pros::c::adi_digital_write(trajector, LOW);
					//FlyWheelVoltage = (1 / .9) * FlyWheelVoltage;
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
