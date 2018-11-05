/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Drive/DifferentialDrive.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <XboxController.h>
#include <Spark.h>
#include <VictorSP.h>
#include <SpeedControllerGroup.h>
#include "WPILib.h"
#include "AHRS.h"
#include <DoubleSolenoid.h>
#include <Timer.h>
#include <iostream>
#include <Encoder.h>

class Robot: public frc::IterativeRobot {

	AHRS *gyro;

	frc::Timer m_timer;
	frc::Timer m_drive_timer;

	frc::VictorSP m_arm_left { 6 };
	frc::VictorSP m_arm_right { 7 };

	frc::VictorSP m_elevator_1 { 5 };
	frc::VictorSP m_elevator_2 { 4 };

	frc::VictorSP m_leftFrontMotor { 0 };
	frc::VictorSP m_leftRearMotor { 1 };

	frc::SpeedControllerGroup m_left { m_leftFrontMotor, m_leftRearMotor };

	frc::VictorSP m_rightFrontMotor { 2 };
	frc::VictorSP m_rightRearMotor { 3 };

	frc::SpeedControllerGroup m_right { m_rightFrontMotor, m_rightRearMotor };

	frc::DifferentialDrive m_robotDrive { m_left, m_right };

	frc::Joystick m_stick { 0 };

	// Initiating the XBox controller(s)

	frc::XboxController driver_controller { 0 };
	frc::XboxController button_controller { 1 };

	frc::AnalogInput ultra_1 { 0 };

public:

	void RobotInit() {

		try {
			gyro = new AHRS(I2C::Port::kOnboard);
		} catch (std::exception& ex) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}

		m_timer.Start();
		CameraServer::GetInstance()->StartAutomaticCapture();
		prefs = Preferences::GetInstance();
		pneumatics_enable = prefs->GetBoolean("PneumaticsEnable", false);
		if (pneumatics_enable) {
			dblSolenoid = new DoubleSolenoid(0, 1);
		}

	}

	void DisabledInit() {
		m_robotDrive.ArcadeDrive(0.0, 0.0);
	}

	void DisabledPeriodic() {
		float angle = -gyro->GetAngle();
		SmartDashboard::PutNumber("Disabled Angle", angle);

		//SmartDashboard Keys
		drive_multiplier = prefs->GetDouble("DriveMultiplier", 0.75);
		turn_multiplier = prefs->GetDouble("TurnMultiplier", 0.70);
		climber_multiplier = prefs->GetDouble("ClimberMultiplier", 1);
		snap_distance_top = prefs->GetDouble("SnapDistanceTop", 0.05);
		snap_distance_bottom = prefs->GetDouble("SnapDistanceBottom", 0.2);
		ramp_multiplier = prefs->GetDouble("RampMultiplier", 0.03);
		box_motor = prefs->GetDouble("BoxMotor", 0.5);
		gyrokp = prefs->GetFloat("gyrokp", 0.03);
		gyroki = prefs->GetFloat("gyroki", 0.06);
		climber_hold = prefs->GetDouble("ClimberHold", 0.10);
		autostr = prefs->GetDouble("AutoStr");
		autoper = prefs->GetString("Auto Period");

		// Show preferences in SmartDashboard (mainly for debugging)
		SmartDashboard::PutNumber("Drive Multiplier:", drive_multiplier);
		SmartDashboard::PutNumber("Turn Multiplier:", turn_multiplier);
		SmartDashboard::PutNumber("Climber Multiplier:", climber_multiplier);
		SmartDashboard::PutNumber("Snap Distance Top:", snap_distance_top);
		SmartDashboard::PutNumber("Snap Distance Bottom:", snap_distance_bottom);
		SmartDashboard::PutNumber("Ramp Multiplier:", ramp_multiplier);
		SmartDashboard::PutBoolean("Pneumatics Enabled:", pneumatics_enable);
		SmartDashboard::PutNumber("Auto Time:", auto_time);
		SmartDashboard::PutNumber("Auto Amount:", auto_amount);
		SmartDashboard::PutNumber("Box Motor Multiplier:", box_motor);
		SmartDashboard::PutNumber("Gyro kP:", gyrokp);
		SmartDashboard::PutNumber("Climber Hold:", climber_hold);
		SmartDashboard::PutNumber("Gyro kI:", gyroki);
		SmartDashboard::PutNumber("Auto Straight:", autostr);
		SmartDashboard::PutString("Auto Period", autoper);

		//Robot Initiate
		m_robotDrive.ArcadeDrive(0.0, 0.0);
		m_elevator_1.Set(-climber_hold);
		Wait(1);
	}

	void AutonomousInit() {
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if (gameData.length() > 0) {
			if (gameData[0] == 'L') {
				direction = -1;
			} else {
				direction = 1;
			}
		}
		m_timer.Reset();
		m_timer.Start();
		m_drive_timer.Reset();
		gyro->ZeroYaw();
		m_drive_timer.Start();
		dblSolenoid->Set(DoubleSolenoid::Value::kReverse);
	}

	void AutonomousPeriodic() {
		time = m_timer.Get();
		float angle = gyro->GetAngle();
		if (autoper == "mid" || autoper == "Mid"){
			//Middle Auto
			switch (auto_status) {
			case -1:
				//Initiate
				m_arm_left.Set(-0.75);
				m_arm_right.Set(0.75);
				m_drive_timer.Reset();
				auto_status++;
				break;
			case 0:
				//Raise Elevator
				m_elevator_1.Set(-.75);
				std::cout << m_drive_timer.Get() << std::endl;
				if (m_drive_timer.Get() > 1.5) {
					auto_status++;
					m_drive_timer.Reset();
					m_arm_left.Set(0);
					m_arm_right.Set(0);
					m_elevator_1.Set(climber_hold);
				}
				break;
			case 1:
				//Drive Forward
				m_robotDrive.ArcadeDrive(.7, -angle * gyrokp);
				if (m_drive_timer.Get() > .6) {
					auto_status++;
				}
				break;
			case 2:
				//Turn toward our switch
				m_robotDrive.ArcadeDrive(0, .5 * direction);
				std::cout << angle << std::endl;
				if (abs(angle) > abs(45 * direction)) {
					auto_status++;
					m_drive_timer.Reset();
					gyro->ZeroYaw();
				}
				break;
			case 3:
				//Drive Forward
				m_robotDrive.ArcadeDrive(.7, -angle * gyrokp);
				if (m_drive_timer.Get() > /*(direction < 0 ? 2.2 : 2)*/1.8) {
					auto_status++;
				}
				break;
			case 4:
				//Turn straight toward switch
				m_robotDrive.ArcadeDrive(0, -.5 * direction);
				if (abs(angle) > abs(15 * direction)) {
					auto_status++;
					m_drive_timer.Reset();
					gyro->ZeroYaw();
				}
				break;
			case 5:\
				//Wait
				m_robotDrive.ArcadeDrive(0 , 0);
				if (m_drive_timer.Get() > 1) {
					auto_status++;
					m_drive_timer.Reset();
				}
				break;
			case 6:
				//Drive Forward
				m_robotDrive.ArcadeDrive(.5, 0);
				if (m_drive_timer.Get() > 0.55) {
					auto_status++;
					m_drive_timer.Reset();
				}
				break;
			case 7:
				//Drop Box
				m_arm_left.Set(0.6);
				m_arm_right.Set(-0.6);
				if (m_drive_timer.Get() > 2) {
					auto_status++;
					m_arm_left.Set(0);
					m_arm_right.Set(0);
				}
				break;
			case 8:
				//End
				break;
			}
			return;
		}

		else if (autoper == "Left" || autoper == "Right" || autoper == "left" || autoper == "right") {

			switch (auto_status) {
			case -1:
				//Initiate
				m_arm_left.Set(-0.75);
				m_arm_right.Set(0.75);
				m_drive_timer.Reset();
				auto_status++;
				break;
			case 0:
				//Raise Elevator
				m_elevator_1.Set(-.75);
				std::cout << m_drive_timer.Get() << std::endl;
				if (m_drive_timer.Get() > 1.5) {
					auto_status++;
					m_drive_timer.Reset();
					m_arm_left.Set(0);
					m_arm_right.Set(0);
					m_elevator_1.Set(climber_hold);
				}
				break;
			case 1:
				//Drive Forward
				m_robotDrive.ArcadeDrive(.7, -angle * gyrokp);
				if (m_drive_timer.Get() > 2.75) {
					auto_status++;
					m_robotDrive.ArcadeDrive(0, 0);
					m_drive_timer.Reset();
				}
				break;
			case 2:
				//If on left and left switch is ours turn
				if ((autoper == "Left" || autoper == "left") && direction == -1) {
					m_robotDrive.ArcadeDrive(0, .5);
					if (gyro->GetAngle() > 90) {
						auto_status++;
						m_drive_timer.Reset();
						gyro->ZeroYaw();
					}
				}
				//If on right and right switch is ours turn
				else if ((autoper == "Right" || autoper == "right") && direction == 1) {
					m_robotDrive.ArcadeDrive(0, -.5);
					if (gyro->GetAngle() < -90) {
						auto_status++;
						m_drive_timer.Reset();
						gyro->ZeroYaw();
					}
				}
				else{
					//If the switch isn't ours end
					auto_status = 6;
				}
				break;
			case 3:
				//Wait for .75 seconds
				m_robotDrive.ArcadeDrive(0 , 0);
				if (m_drive_timer.Get() > .75){
					auto_status++;
					m_drive_timer.Reset();
				}

				break;
			case 4:
				//Drive Forward
				m_robotDrive.ArcadeDrive(.5 , -angle * gyrokp);
				if(m_drive_timer.Get() > 1.25){
					auto_status++;
					m_robotDrive.ArcadeDrive(0 , 0);
					m_drive_timer.Reset();
				}
				break;
			case 5:
				//Release Cube
				m_arm_left.Set(0.6);
				m_arm_right.Set(-0.6);
				if (m_drive_timer.Get() > 2) {
					auto_status++;
					m_arm_left.Set(0);
					m_arm_right.Set(0);
				}
				break;
			case 6:
				//End
			return;
			}
		}

		//Drive Straight
		else {
			time = m_timer.Get();
			if (time < 3) {
				m_robotDrive.ArcadeDrive(.65, 0);
				time = m_timer.Get();
			} else {
				m_robotDrive.ArcadeDrive(0, 0);
			}
		}
	}

	void TeleopInit() {
		if (pneumatics_enable) {
			dblSolenoid->Set(DoubleSolenoid::Value::kReverse);
		}
	}

	void TeleopPeriodic() {

		//Puts the Gyro angle on smart dashboard
		float angle = gyro->GetAngle();
		SmartDashboard::PutNumber("Teleop Angle", angle);
		getInput();

		//If Operator controller or Drive controller presses these buttons drive speed reduced
		if (button_y_2 || button_lb || button_rb) {
			if (force_slow > 0 && force_slow > 0.7) {
				force_slow -= 0.03;
			}
		} else {
			if (force_slow < 1) {
				force_slow += 0.03;
			}
		}

		//Drive forward and driver input builds to target
		target = (input_lt - input_rt) * drive_multiplier * force_slow;
		if (target - current < snap_distance_top
				&& target - current > -snap_distance_top)
			current = target;
		if (input_lt == 0.0 && input_rt == 0.0) { // When there's no input, stop the bus
			current += (target - current) * (ramp_multiplier + 0.1);
		}

		//Slow Down
		else {
			current += (target - current) * ramp_multiplier;

			// If in this limbo area, snap out of it!
			if (current < 0 && current > -snap_distance_bottom) {
				if (input_rt != 0.0) {    // Right Trigger held, pull up
					current = -snap_distance_bottom;
				} else {    // Right trigger isn't held, snap down to zero
					current = 0;
				}
			} else if (current > 0 && current < snap_distance_bottom) {
				if (input_lt != 0.0) {    //Left Trigger held, pull up
					current = snap_distance_bottom;
				} else {    //Left trigger isn't held, snap down to zero
					current = 0;
				}
			}
		}

		//If operator presses x elevator hold
		if (button_x_prev != button_x && button_x) {
			holdClimber = !holdClimber;
		}

		//If operator is pressing Right Bumber Raise elevator
		if (button_rb) {
			m_elevator_1.Set(-climber_multiplier);
			holdClimber = false;
		}

		//If operator is pressing Left Bumber Raise elevator
		else if (button_lb) {
			m_elevator_1.Set(climber_multiplier / 2.5);
			holdClimber = false;
		}

		//If operator presses x drop elevator
		else if (holdClimber) {
			m_elevator_1.Set(0);
		}

		//Otherwise elevator set to climber hold
		else {
			m_elevator_1.Set(climber_hold);
			holdClimber = false;
		}

		//Sets button_x_prev to button_x
		button_x_prev = button_x;

		//Drive is -current to go forward and turn is the input of left stick x axis * turn multiplier
		m_robotDrive.ArcadeDrive(-current,
				input_left_stick_x * turn_multiplier);

		//Put on dashboard
		SmartDashboard::PutNumber("Target Speed:", -target);
		SmartDashboard::PutNumber("Current Speed:", -current);
		SmartDashboard::PutNumber("Current Stick:", input_left_stick_x);
		SmartDashboard::PutNumber("Current Turn:",input_left_stick_x * (-target));
		SmartDashboard::PutNumber("Ultrasonic 1:", (ultra_1.GetVoltage() / 0.001) / 45);
		SmartDashboard::PutNumber("Force Slow", force_slow);

		if (pneumatics_enable) {

			//A opens the arm
			if (button_a) {
				dblSolenoid->Set(DoubleSolenoid::Value::kForward);
			}

			//B closes the arm
			if (button_b) {
				dblSolenoid->Set(DoubleSolenoid::Value::kReverse);
			}
		}

		//If operator presses right trigger suck in the box
		if (input_b_rt != 0) {
			m_arm_left.Set(box_motor * -input_b_rt);
			m_arm_right.Set(-box_motor * -input_b_rt);
		}

		//If operator presses left trigger shoot out the box
		else if (input_b_lt != 0) {
			m_arm_left.Set(-box_motor * -input_b_lt);
			m_arm_right.Set(box_motor * -input_b_lt);
		}

		//Otherwise arm is 0
		else {
			m_arm_left.Set(0.0);
			m_arm_right.Set(0.0);
		}

	}

private:

	bool pneumatics_enable;
//Robot Preferences from SmartDashboard
	Preferences *prefs;

//Value from 0.0 to 1.0, reduces motor speed to prevent issues during testing.
	double drive_multiplier; // Default 0.75
	double snap_distance_top; //Default is 0.05
	double snap_distance_bottom; //Default is 0.2
	double ramp_multiplier; //Default is 0.03
	double turn_multiplier; // Default 0.50
	double climber_multiplier; // Default 0.3
	double box_motor;
	double climber_hold;

	int time, time_2 = 0;

//Autonomous Values
	int counter;
	std::string gameData;
	std::string autoper;
	int robo_pos;
	double autostr;
	double auto_time;
	double auto_amount;
	float gyrokp;
	float gyroki;
	int leftRightNegative = 0;
	int direction = 0;
	float stageLength[11];
	int auto_status = -1;

	float autoInt;
	float autoP;
	float autoI;
	float autoD;
	float autoError, autoSetpoint, autoRCW;

//Pneumatics
	DoubleSolenoid *dblSolenoid;

//Drive Values
	double current = 0.0, target = 0.0;

	float force_slow = 1.0;

//Analog stick input values
	double input_left_stick_y, input_right_stick_y, input_left_stick_x,
			input_right_stick_x;

//Trigger Values
	double input_lt, input_rt, input_b_lt, input_b_rt;

//Face button values
	bool button_a, button_b, button_x, button_y, button_y_2;

	bool button_x_prev, holdClimber, ignoreNext = false;

//Bumpers

	bool button_lb, button_rb;

	int count_left;

//Encoder *sampleEncoder;

	void getInput() {
		//Get Sticks
		input_left_stick_y = driver_controller.GetY(
				GenericHID::JoystickHand::kLeftHand);
		input_right_stick_y = driver_controller.GetY(
				GenericHID::JoystickHand::kRightHand);
		input_left_stick_x = driver_controller.GetX(
				GenericHID::JoystickHand::kLeftHand);
		input_right_stick_x = driver_controller.GetX(
				GenericHID::JoystickHand::kRightHand);

		//Get Triggers
		input_lt = driver_controller.GetTriggerAxis(
				GenericHID::JoystickHand::kLeftHand);
		input_rt = driver_controller.GetTriggerAxis(
				GenericHID::JoystickHand::kRightHand);

		//Get Triggers
		input_b_lt = button_controller.GetTriggerAxis(
				GenericHID::JoystickHand::kLeftHand);
		input_b_rt = button_controller.GetTriggerAxis(
				GenericHID::JoystickHand::kRightHand);

		button_a = button_controller.GetAButton();
		button_b = button_controller.GetBButton();
		button_x = button_controller.GetXButton();
		button_y = button_controller.GetYButton();
		button_y_2 = button_controller.GetYButton();

		//Get Buttons
		button_lb = button_controller.GetBumper(
				GenericHID::JoystickHand::kLeftHand);
		button_rb = button_controller.GetBumper(
				GenericHID::JoystickHand::kRightHand);

	}
}
;

START_ROBOT_CLASS(Robot)
