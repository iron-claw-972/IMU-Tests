package org.usfirst.frc.team9972.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	ADIS16448_IMU imu = new ADIS16448_IMU();
	BuiltInAccelerometer accel = new BuiltInAccelerometer();

	Joystick leftJoy = new Joystick(0);
	Joystick rightJoy = new Joystick(1);
	Joystick operatorJoy = new Joystick(2);

	Victor backRightMotor = new Victor(0);
	Victor backLeftMotor = new Victor(1);
	Victor frontRightMotor = new Victor(2);
	Victor frontLeftMotor = new Victor(3);

	double setAngle_Straight = 0.0;
	double setAngle_Turn = 0.0;
	double turnSpeed = .5;

	double driveStraightSpeed = 0.0;

	double[] acceleration = new double[3];

	RobotDrive drive = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

	boolean leftPressedLastTime = false;
	boolean driveStraight = false;

	boolean right1PressedLastTime = false;
	boolean right2PressedLastTime = false;
	boolean right3PressedLastTime = false;

	boolean right1IsPressed = false;

	boolean tankDrive = true;
	double initAngleX, initAngleY, initAngleZ;
	double lastAngleZ = 0;
	double velocityX = 0;
	long lastTime1 = 0;
	long lastTime2 = 0;
	double positionX = 0;
	boolean init180 = false;

	
	public void teleopInit() {
		imu.calibrate();
		initAngleX = imu.getAngleX();
		initAngleY = imu.getAngleY();
		initAngleZ = imu.getAngleZ();
		/*
		 * Graph gets all messed up if we move our robot before the graph
		 * calibrates. We need to wait approx 3 seconds (not positive; it would
		 * be best to remeasure) before moving the robot so the graph has time
		 * to calibrate
		 */

	}

	public void autonomousPeriodic() {
		SmartDashboard.putData("IMU", imu);
		SmartDashboard.putNumber("Angle Y", imu.getAngleY() / 4);
		SmartDashboard.putNumber("Angle X", imu.getAngleX() / 4);
		SmartDashboard.putNumber("Angle Z", imu.getAngleZ() / 4);
		SmartDashboard.putNumber("Accel X", imu.getAccelX());
		SmartDashboard.putNumber("Accel Y", imu.getAccelY());
		SmartDashboard.putNumber("Accel Z", imu.getAccelZ());
	}

	public void teleopPeriodic() {
		acceleration[0] = imu.getAccelX();// IMU not tested yet with
											// acceleration
		acceleration[1] = imu.getAccelY();
		acceleration[2] = imu.getAccelZ();
		SmartDashboard.putData("IMU", imu);
		SmartDashboard.putNumber("X Acceleration", acceleration[0]);
		SmartDashboard.putNumber("Y Acceleration", acceleration[1]);
		SmartDashboard.putNumber("Z Acceleration", acceleration[2]);
		SmartDashboard.putNumber("Angle X", imu.getAngleX() / 4);// For all
																	// angles,
																	// scale by
																	// dividing
																	// by 4 (ex:
																	// a 90
																	// degree
																	// turn by
																	// the robot
																	// returns a
																	// Z angle
																	// change of
																	// 360)
		SmartDashboard.putNumber("Angle Y", imu.getAngleY() / 4);
		SmartDashboard.putNumber("Angle Z", imu.getAngleZ() / 4);
		SmartDashboard.putNumber("On Board Accel X", accel.getX());
		SmartDashboard.putNumber("On Board Accel Y", accel.getY());
		SmartDashboard.putNumber("On Board Accel Z", accel.getZ());
		SmartDashboard.putNumber("last angle z", lastAngleZ);

		

		// right1IsPressed = rightJoy.getRawButton(1);
		// if (right1IsPressed && !right1PressedLastTime) {
		// tankDrive = !tankDrive;
		// }
		// right1PressedLastTime = right1IsPressed;

		if (tankDrive) {
			drive.tankDrive(-leftJoy.getY(), -rightJoy.getY()); // joysticks
																// wrong sign
		}

		// rect integration begins here?
		double velX = acceleration[0] * (double) (System.currentTimeMillis() - lastTime1);
		velocityX += velX;
		SmartDashboard.putNumber("Velocity X", velocityX);
		lastTime1 = System.currentTimeMillis();

		double posX = velocityX * (double) (System.currentTimeMillis() - lastTime2);
		positionX += posX;
		SmartDashboard.putNumber("Position X", positionX);
		lastTime2 = System.currentTimeMillis();
		//

		boolean right2IsPressed = rightJoy.getRawButton(2);
		if (right2IsPressed && !right2PressedLastTime) {
			if(init180 == false){
				lastAngleZ = (imu.getAngleZ() / 4) + 180;// target angle
				init180 = true;
			}
			else if ((imu.getAngleZ() / 4) < lastAngleZ) {// used to be while, but
														// now an if statement
														// and doesn't work
				drive.tankDrive(turnSpeed, -turnSpeed);
		//		System.out.println("turning 180");// for testing
			}
			else{
				init180 = false;
			}
		}
		right2PressedLastTime = right2IsPressed;

		boolean right3IsPressed = rightJoy.getRawButton(3);// doesn't work yet
		if (right3IsPressed && !right3PressedLastTime) {// should return robot
														// to original Z front
														// facing position from
														// teleOpInit
			double rangedAngleZ = imu.getAngleZ() / 4 % 360.0;// ranged angle is
																// an angle
																// within the
																// range of -360
																// to 360
			if (rangedAngleZ < 0) {
				rangedAngleZ = 360 - rangedAngleZ;
			}

			SmartDashboard.putNumber("Ranged Angle Z", rangedAngleZ);

			if (rangedAngleZ != initAngleZ) {

				if (rangedAngleZ < initAngleZ) {
					drive.tankDrive(turnSpeed, -turnSpeed);
				} else if (rangedAngleZ > initAngleZ) {
					drive.tankDrive(-turnSpeed, turnSpeed);
				}
			}
		}
		right3PressedLastTime = right3IsPressed;
	}
}