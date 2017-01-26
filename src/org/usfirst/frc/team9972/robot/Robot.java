package org.usfirst.frc.team9972.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	ADIS16448_IMU imu = new ADIS16448_IMU();

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
	boolean right2IsPressed = false;
	boolean right3IsPressed = false;
	
	boolean tankDrive = true;
	double initAngleX, initAngleY, initAngleZ;
	
	public void teleopInit() {
		imu.calibrate();
		initAngleX = imu.getAngleX();
		initAngleY = imu.getAngleY();
		initAngleZ = imu.getAngleZ();
		/*
		 * Graph gets all messed up if we move our robot before the graph calibrates.
		 * We need to wait approx 3 seconds (not positive; it would be best to remeasure)
		 * before moving the robot so the graph has time to calibrate
		*/
		
	}
	public void autonomousPeriodic() {
		  SmartDashboard.putData("IMU", imu); 
		  SmartDashboard.putNumber("Angle Y", imu.getAngleY()/4);
		  SmartDashboard.putNumber("Angle X", imu.getAngleX()/4);
		  SmartDashboard.putNumber("Angle Z", imu.getAngleZ()/4);
		  SmartDashboard.putNumber("Accel X", imu.getAccelX());
		  SmartDashboard.putNumber("Accel Y", imu.getAccelY());
		  SmartDashboard.putNumber("Accel Z", imu.getAccelZ());
		}
	
	public void teleopPeriodic() {
	    acceleration[0] = imu.getAccelX();// IMU not tested yet with acceleration
		acceleration[1] = imu.getAccelY();
		acceleration[2] = imu.getAccelZ();
		SmartDashboard.putData("IMU", imu); 		
		SmartDashboard.putNumber("X Acceleration", acceleration[0]);
		SmartDashboard.putNumber("Y Acceleration", acceleration[1]);
		SmartDashboard.putNumber("Z Acceleration", acceleration[2]);
		SmartDashboard.putNumber("Angle X", imu.getAngleX()/4);//For all angles, scale by dividing by 4 (ex: a 90 degree turn by the robot returns a Z angle change of 360)
		SmartDashboard.putNumber("Angle Y", imu.getAngleY()/4);
		SmartDashboard.putNumber("Angle Z", imu.getAngleZ()/4);
	  //  SmartDashboard.putNumber("last angle z", lastAngleZ);*

		
//		right1IsPressed = rightJoy.getRawButton(1);
//		if (right1IsPressed && !right1PressedLastTime) {
//			tankDrive = !tankDrive;
//		}
//		right1PressedLastTime = right1IsPressed;
//
//		
//		if (tankDrive){
			drive.tankDrive(-leftJoy.getY(), -rightJoy.getY()); //joysticks wrong sign
	//	}

		right2IsPressed = rightJoy.getRawButton(2);
		if (right2IsPressed && !right2PressedLastTime) {
			double lastAngleZ = (imu.getAngleZ()/4) + 180;//target angle
			if((imu.getAngleZ()/4) < lastAngleZ){//used to be while, but now an if statement and doesn't work
				  drive.tankDrive(turnSpeed, -turnSpeed);
			//	  System.out.println("turning 180");//for testing
			/*	  SmartDashboard.putData("IMU", imu); 
				  SmartDashboard.putNumber("Angle Y", imu.getAngleY()/4);//For all angles, scale by dividing by 4 (ex: a 90 degree turn by the robot returns a Z angle change of 360)
				  SmartDashboard.putNumber("Angle X", imu.getAngleX()/4);
				  SmartDashboard.putNumber("Angle Z", imu.getAngleZ()/4);
				  SmartDashboard.putNumber("Acceleration X", imu.getAccelX());
				  SmartDashboard.putNumber("Acceleration Y", imu.getAccelY());
				  SmartDashboard.putNumber("Acceleration Z", imu.getAccelZ());*/
				  SmartDashboard.putNumber("last angle z", lastAngleZ);
				  
			}
		}
		right2PressedLastTime = right2IsPressed;
		
		right3IsPressed = rightJoy.getRawButton(3);//doesn't work yet
		if (right3IsPressed && !right3PressedLastTime) {//should return robot to original Z front facing position from teleOpInit
			double rangedAngleZ = imu.getAngleZ()/4 % 360.0;//ranged angle is an angle within the range of -360 to 360
			if(rangedAngleZ < 0){
				rangedAngleZ = 360 - rangedAngleZ;
			}
			
			SmartDashboard.putNumber("Ranged Angle Z", rangedAngleZ);
			
			if(rangedAngleZ != initAngleZ){
				
				if(rangedAngleZ < initAngleZ){
					drive.tankDrive(turnSpeed, -turnSpeed);
				}
				else if(rangedAngleZ > initAngleZ){
					drive.tankDrive(-turnSpeed, turnSpeed);
				}
			}
		}
		right3PressedLastTime = right3IsPressed;
	}
}