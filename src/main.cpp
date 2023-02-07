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

enum AutonMode {
	AutonLeft = 1,
	AutonRight = 2,
	AutonSkills = 3,
	AutonNone = 0,
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

	void SetFlywheelVoltage(unsigned int voltage)
	{
		FlyWheel1.move_voltage(-voltage);
		Intake.move_velocity(-voltage);
	}

	void ShootDiskAccurate_old(unsigned int speed, int delay)
	{
        SetFlywheelVelocity(speed);
        pros::c::delay(delay);

        for (int i = 0; i < 200; i++) {
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

	void ShootDiskAccurate(unsigned int speed)
	{
		// max time we wait for flywehee to reach desired speed, using SetFlywheelVelocity() flow
		auto waitTimeMax = 400;
		// If speed is achieved sooner than above timeout, we wait extra 20 cycles before switching
		// to waiting using voltage-based algorithm 
		auto autoextraWaitAfterReachingSpeed = 70;
		// Time we wait using voltage-setting algorithm
		auto settlementTime = 180;
		// amount of time we wait after extending piston before retrieving it back
		auto pistonExtendedTime = 40;

		SetFlywheelVelocity(speed);

		unsigned int counter = 0;
		double speedsum = 0;

		while (true) {

			// Further out we are - the more voltage we need to faster get there
			// But once we reach the speed, we need some power to simply maintain momentum
			auto vel = getFlywheelVelocity();

			// Are we still waiting for flywheel to speed up?
			if (waitTimeMax > 0)
			{
				waitTimeMax--;
				// if we reached desired speed, we are moving to settlement period
				// But give it 200ms to work through with velocity-based algorithm
				if (vel >= speed && waitTimeMax >= autoextraWaitAfterReachingSpeed)
				{
					log("--- Waited %d ---\n", (400-waitTimeMax) * 10);
					waitTimeMax = autoextraWaitAfterReachingSpeed;
				}
				if (waitTimeMax == 0)
					log("--- Voltage-based --- \n");
			} else {
				int voltage = speed * 126;
				if (vel < speed)
					voltage += 30 * (speed - vel);

				// Max is +-12,000
				if (voltage >= 12000)
					voltage = 12000;

				// printf("%.1f\n", vel);
				counter++;
				speedsum += vel;

				FlyWheel1.move_voltage(-voltage);
				Intake.move_voltage(-voltage);
			}

			// settlement period - just wait, so specific action
			if (waitTimeMax == 0 && settlementTime > 0)
			{
				log("%.1f\n", vel);
				settlementTime--;
				if (settlementTime == 0)
					log("---- Shoot! ---\n");
			}

			// shooting
			if (settlementTime == 0 && pistonExtendedTime > 0) {
				pistonExtendedTime--;
				pros::c::adi_digital_write(ShootPort, HIGH);
				log("%.1f\n", vel);
			}

			if (pistonExtendedTime == 0)
				break;

			pros::c::delay(10);
		}

		printf("%.2f\n", speedsum / counter);

		SetFlywheelVelocity(speed);
		pros::c::adi_digital_write(ShootPort, LOW);
		log("\n\n");
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
		SetFlywheelVoltage(8600);

		//Turn to aim at goal
		Turn(-18, 100);
		pros::c::delay(400);

		//Shoot the two preloads
		ShootDiskAccurate_voltage(8600, 2000);

		ShootDiskAccurate_voltage(8600, 1000);

		//Start Roller
		SetRollerVelocity(90);

		//Turn back to start
		Turn(18, 100);
		pros::c::delay(250);


		//Move back towards roller
		Move(175, -70, -70, 350);
		pros::c::delay(50);

		//Move forwards from roller after it's turned
		Move(100, 100, 100, 1000);
		pros::c::delay(50);
		
		//prep flywheel
		SetFlywheelVoltage(8000);

		//Turn towards stack of discs
		Turn(40, 100);
		pros::c::delay(500);

		//Pick up discs
		Move(1200, 100, 100, 5000);
		pros::c::delay(50);

		//Turn towards goal
		Turn(-78, 100);
		pros::c::delay(600);

		//Shoot three discs
		ShootDiskAccurate_voltage(8000, 1000);

		ShootDiskAccurate_voltage(8000, 1000);

		ShootDiskAccurate_voltage(8000, 1000);

	}

	void runRight() {
		// prep flywheel
		SetFlywheelVoltage(9200);

		Move(350, 100, 100, 3000);
		pros::c::delay(500);

		Turn(23.5, 100);
	
		ShootDiskAccurate_voltage(9100, 2000);

		ShootDiskAccurate_voltage(9100, 1500);

		ShootDiskAccurate_voltage(9100, 1500);

		// prep for future shots & disk pick up
		SetFlywheelVoltage(9100);

		// turn towards 2 disks
		Turn(-61, 75);
		pros::c::delay(1000);

		// pick up 2 disks
		Move(700, 100, 100, 3000);
		pros::c::delay(500);

		// Move backwards toward roller
		Move(1400, -120, -120, 3000);
		pros::c::delay(500);

		// turn towards roller
		Turn(45, 100);
		pros::c::delay(1000);

		// turn roller
		SetRollerVelocity(90);
		Move(130, -100, -100, 1000);
		pros::c::delay(500);

		// move out, prep to shoot
		Move(50, 100, 100, 400);

		/*
		ShootDiskAccurate_voltage(9100, 1000);
		ShootDiskAccurate_voltage(9100, 1000);*/
	}

	void runSkills() {
		// prep flywheel
		
		/*SetFlywheelVelocity(82);

		Turn(-13.5, 100);
		pros::c::delay(200);

		ShootDiskAccurate_old(82, 2000);

		ShootDiskAccurate_old(84, 1000);

		SetRollerVelocity(90);
	
		Turn(13.5, 100);
		pros::c::delay(200);

		Move(180, -70, -70, 1000);
		pros::c::delay(200);

		Move(100, 100, 100, 1000);
		pros::c::delay(50);

		Turn(50, 100);
		pros::c::delay(250);

		SetFlywheelVelocity(66);
		pros::c::delay(75);

		Move(600, 50, 50, 2000);
		pros::c::delay(50);

		Move(600, 40, 40, 3000);
		pros::c::delay(50);

		Turn(-66, 100);
		pros::c::delay(350);

		ShootDiskAccurate(79);
		pros::c::delay(50);

		ShootDiskAccurate(79);
		pros::c::delay(50);

		ShootDiskAccurate(79);
		pros::c::delay(50);

		SetRollerVelocity(0);
		pros::c::delay(100);

		Turn(66, 100);
		pros::c::delay(400);

		Move(1200, -50, -50, 4000);
		pros::c::delay(100);*/

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
		const int defaultFlyWheelSpeed = -69;
		int FlyWheelSpeed = defaultFlyWheelSpeed;
		int FlyWheelOn = 0;

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
				FlyWheelSpeed = defaultFlyWheelSpeed;
			}

			// Flywheel is powered, reverse
			if (master.get_digital_new_press(DIGITAL_Y))
			{
				FlyWheelSpeed = -defaultFlyWheelSpeed;
			}

			//Flywheel is stopped
			if (master.get_digital_new_press(DIGITAL_B))
			{
				FlyWheelSpeed = 0;
			}

			// Flywheel speed is high
			if (master.get_digital_new_press(DIGITAL_X))
			{
				FlyWheelSpeed = -73;
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
