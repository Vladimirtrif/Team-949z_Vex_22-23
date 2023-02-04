#include "main.h"
#include "pros/adi.h"
#include "pros/llemu.hpp"
#include "pros/rotation.hpp"

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

	pros::Rotation FlyWheelSensor{FlyWheelSensorPort};

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

	auto getFlywheelVelocity()
	{
		return -FlyWheelSensor.get_velocity();
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

	void SetFlywheelVoltageCore(unsigned int voltage)
	{
		FlyWheel1.move_voltage(-voltage);
		Intake.move_velocity(-voltage);
	}

	void ShootDiskAccurate_voltage(unsigned int voltage, int delay)
	{
        SetFlywheelVoltageCore(voltage);
        pros::c::delay(delay);

        ShootDisk();
	}

	unsigned int _flywheelTargetVoltage = 0;
	int _flywheelVelocity = 0;
	int _flywheelVelocityDiff = 0;

	void SetFlyWheelTargetVoltage(unsigned int targetVoltage) {
		_flywheelTargetVoltage = targetVoltage;
		SetFlywheelVoltageCore(targetVoltage);

		// ensure we will not exit early because flywheel is slowing down at the start
		_flywheelVelocityDiff = 10;
		_flywheelVelocity = getFlywheelVelocity();
}

	void FlyWheelCycle()
	{
		unsigned int targetSpeed = _flywheelTargetVoltage / 0.62 - 1286;
		// int _flywheelTargetVoltage = (targetSpeed + 1286) * 0.62;

		int velCurr = getFlywheelVelocity();
		int velLast = _flywheelVelocity;
		_flywheelVelocity = _flywheelVelocity * 0.85 + 0.15 * velCurr;
		_flywheelVelocityDiff = _flywheelVelocityDiff * 0.9 + 0.1 * (_flywheelVelocity - velLast);

		int distance = targetSpeed - _flywheelVelocity;

		unsigned int power = _flywheelTargetVoltage;
		if (distance > 0) {
			power += distance / 50;
		}

		SetFlywheelVoltageCore(power);

		log("%d %d %d\n", velCurr, power, _flywheelVelocityDiff);
	}

	// Maintains flywheel speed
	void delay(unsigned int time)
	{
		for (int i = 0; i < time / 10; i++) {
			FlyWheelCycle();
			pros::c::delay(10);
		}
	}

	void ShootDiskAccurate()
	{	
		delay(100);

		// if we see some slow down, that means we are at target (not growing) speed, start countdown
		// do not bounce speeds, just maintain target speed and let time stabilize speed.
		while (_flywheelVelocityDiff > -5) {
			FlyWheelCycle();
			pros::c::delay(10);
		}

		delay(200);

		pros::c::adi_digital_write(ShootPort, HIGH);
		delay(300);

		pros::c::adi_digital_write(ShootPort, LOW);
		delay(300);
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
			delay(10);
			counter = counter + 10;
		}

		SetDrive(0, 0);
		delay(100);
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
					log("%d  %d\n", obj.width, obj.top_coord + obj.height);
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

		SetFlyWheelTargetVoltage(8300);
		delay(350);

		Turn(-18, 100);
		delay(200);

		/*ShootDiskAccurate_old(88, 2000);

		ShootDiskAccurate_old(88, 1000);*/

		ShootDiskAccurate();

		ShootDiskAccurate();

		SetRollerVelocity(90);

		Turn(18, 100);
		delay(250);

		Move(175, -70, -70, 350);
		delay(50);

		Move(100, 100, 100, 1000);
		delay(50);

		Turn(40, 100);
		delay(250);

		SetFlyWheelTargetVoltage(7500);
		delay(75);

		Move(600, 50, 50, 2000);
		delay(50);

		Move(600, 40, 40, 3000);
		delay(50);

		Turn(-67, 100);
		delay(350);

		ShootDiskAccurate();

		ShootDiskAccurate();

		ShootDiskAccurate();
	}

	void runRight() {
		// prep flywheel
		/*SetFlyWheelTargetVoltage(9751);
		delay(500);


		Turn(19.5, 100);
		delay(350);

		ShootDiskAccurate();

		ShootDiskAccurate();

		Turn(65.5, 100);
		delay(750);

		Move(275, 100, 100, 3000);
		delay(500);

		Turn(-75, 100);
		delay(450);

		SetRollerVelocity(90);
		delay(250);

		Move(140, -70, -70, 400);
		delay(50);

		Move(70, 100, 100, 10000);
		delay(50);

		Turn(-45.5, 100);
		delay(500);

		SetFlyWheelTargetVoltage(7500);
		delay(75);

		Move(1800, 100, 100, 10000);
		delay(200);

		Turn(88, 100);
		delay(500);

		ShootDiskAccurate();

		ShootDiskAccurate();

		ShootDiskAccurate();*/

		SetFlyWheelTargetVoltage(9100);
		delay(500);

		Move(350, 100, 100, 3000);
		delay(1000);

		Turn(23.5, 100);
		delay(1000);

		ShootDiskAccurate();

		ShootDiskAccurate();

		ShootDiskAccurate();

		SetFlyWheelTargetVoltage(8000);

		Turn(-69.5, 100);
		delay(1000);

		Move(650, 100, 100, 3000);
		delay(1000);

		Turn(87.5, 100);
		delay(1000);

		ShootDiskAccurate();

		ShootDiskAccurate();

		Turn(-82.5, 100);
		delay(1000);

		SetRollerVelocity(90);

		Move(2000, -120, -120, 3000);
		delay(1000);

		Move(200, 100, 100, 3000);
		delay(1000);
	}

	void runSkills() {
		// prep flywheel
		
		/*SetFlywheelVelocity(82);

		Turn(-13.5, 100);
		delay(200);

		ShootDiskAccurate_old(82, 2000);

		ShootDiskAccurate_old(84, 1000);

		SetRollerVelocity(90);
	
		Turn(13.5, 100);
		delay(200);

		Move(180, -70, -70, 1000);
		delay(200);

		Move(100, 100, 100, 1000);
		delay(50);

		Turn(50, 100);
		delay(250);

		SetFlywheelVelocity(66);
		delay(75);

		Move(600, 50, 50, 2000);
		delay(50);

		Move(600, 40, 40, 3000);
		delay(50);

		Turn(-66, 100);
		delay(350);

		ShootDiskAccurate(79);
		delay(50);

		ShootDiskAccurate(79);
		delay(50);

		ShootDiskAccurate(79);
		delay(50);

		SetRollerVelocity(0);
		delay(100);

		Turn(66, 100);
		delay(400);

		Move(1200, -50, -50, 4000);
		delay(100);*/

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
		const int defaultFlyWheelSpeed = -59;
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
				FlyWheelSpeed = -67;
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

			pros::c::delay(10);
		}
	}
};

void opcontrol()
{
	OpControl program;
	program.opcontrol();
}
