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

	double[] acceleration = new double[3]; //[0] = X, [1] = Y, [2] = Z
	double turnSpeed = 0.5;
	double lastAccelX = 0;
	double lastVelocityX = 0;
	double positionX = 0;
	double velocityX = 0;
	double lastTimeX = 0;//last time for calculating velocity and position
	public static final double accelThresh = .06;// acceleration Threshold to integrate to be used in a conditional or added as a constant
	
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
	
	double startTime = 0;
	
	public void teleopInit() {
		imu.calibrate();
		initAngleX = imu.getAngleX(); // finds the angles of start position for reference later
		initAngleY = imu.getAngleY();
		initAngleZ = imu.getAngleZ();
		startTime = (double)(System.currentTimeMillis() / 1000);//start time to subtract from current System.currentTimeMillis() for actual time
		/*
		 * Graph gets all messed up if we move our robot before the graph calibrates.
		 * We need to wait approx 3 seconds (not positive; it would be best to remeasure)
		 * before moving the robot so the graph has time to calibrate
		*/
	}
	
	public void autonomousPeriodic() {
//		  SmartDashboard.putData("IMU", imu); 
//		  SmartDashboard.putNumber("Angle Y", imu.getAngleY()/4);
//		  SmartDashboard.putNumber("Angle X", imu.getAngleX()/4);
//		  SmartDashboard.putNumber("Angle Z", imu.getAngleZ()/4);
//		  SmartDashboard.putNumber("Accel X", imu.getAccelX());
//		  SmartDashboard.putNumber("Accel Y", imu.getAccelY());
//		  SmartDashboard.putNumber("Accel Z", imu.getAccelZ());
		}
	
	public void teleopPeriodic() {
		if( imu.getAccelX() * 9.8 > accelThresh || imu.getAccelX() * 9.8 < -accelThresh){ // to avoid integrating tiny accelerations over and over
			velocityX = velocityX + (lastAccelX + imu.getAccelX()) * 4.9 * (((double)(System.currentTimeMillis()/1000) - startTime) - lastTimeX);// or instead of if statement above, add a constant accelThresh if that works
		}
		// add the "acceleration (m/s/s) * time (s)" trapezoid to get velocity (m/s)
		// acceleration is measured in g's, so to get the average of the accelerations in m/s/s, multiply by 9.8 and divide by 2, .032 is the error in the IMU
		lastAccelX = imu.getAccelX();
		positionX = positionX + ((velocityX + lastVelocityX) /2) * (((double)(System.currentTimeMillis()/1000) - startTime) - lastTimeX);
		// add the "velocity (m/s) * time (s)" trapezoid to get position (m)
		lastTimeX = (double)(System.currentTimeMillis()/1000) - startTime;
		
				
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
		SmartDashboard.putNumber("Velocity X", velocityX);
		SmartDashboard.putNumber("Position X", positionX);

		if (tankDrive){
			drive.tankDrive(-leftJoy.getY(), -rightJoy.getY()); //joysticks wrong sign
		}

//		right2IsPressed = rightJoy.getRawButton(2);
//		if (right2IsPressed && !right2PressedLastTime) {
//			double lastAngleZ = imu.getAngleZ()/4;
//			if((imu.getAngleZ()/4) < lastAngleZ + 180){
//				  drive.tankDrive(turnSpeed, -turnSpeed);
//				  SmartDashboard.putData("IMU", imu); 
//				  SmartDashboard.putNumber("Angle Y", imu.getAngleY()/4);//For all angles, scale by dividing by 4 (ex: a 90 degree turn by the robot returns a Z angle change of 360)
//				  SmartDashboard.putNumber("Angle X", imu.getAngleX()/4);
//				  SmartDashboard.putNumber("Angle Z", imu.getAngleZ()/4);
//				  SmartDashboard.putNumber("Acceleration X", imu.getAccelX());
//				  SmartDashboard.putNumber("Acceleration Y", imu.getAccelY());
//				  SmartDashboard.putNumber("Acceleration Z", imu.getAccelZ());
//				  SmartDashboard.putNumber("last angle z", lastAngleZ);
//			}
//		}
//		right2PressedLastTime = right2IsPressed;
	/*	right3IsPressed = rightJoy.getRawButton(3);//doesn't work yet
		if (right3IsPressed && !right3PressedLastTime) {//should return robot to original Z front facing position from teleOpInit
			double rangedAngleZ = imu.getAngleZ()/4 % 360.0;//ranged angle is an angle within the range of -360 to 360
			while(rangedAngleZ != initAngleZ){
				if(rangedAngleZ < initAngleZ){
					drive.tankDrive(turnSpeed, -turnSpeed);
				}
				else if(rangedAngleZ > initAngleZ){
					drive.tankDrive(-turnSpeed, turnSpeed);
				}
			}
		}
		right3PressedLastTime = right3IsPressed;*/
	}
}