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

	double currAccel = 0.0; //acc in Y direction (parallel to driving)
	double currAngle = 0.0; //degrees in XY-plane
	double currTime = 0.0; //time in secs
	double currVel = 0.0; //velocity in m/s
	double currX = 0.0; //position in m
	double currY = 0.0; //position in m
	
	double initTime = 0.0; //in seconds
	double initAngle = 0.0; //in degrees
	double initAccel = 0.0; //in m/s^2
	
	double prevTime = 0.0;
	double prevVel = 0.0;
	double prevX = 0.0;
	double prevY = 0.0;
	
	double deltaTime = 0.0;
	
	RobotDrive drive = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
	
	/* Graph gets all messed up if we move our robot before the graph calibrates.
	We need to wait approx 3 seconds (not positive; it would be best to remeasure)
	before moving the robot so the graph has time to calibrate */
	
	public void teleopInit() {
		imu.calibrate();
		initAngle = - (imu.getAngleZ() / 4); //negative because we want positive in counterclockwise direction
		initAccel = - (imu.getAccelY() * 9.8); //negative because front of robot is in other direction
		initTime = ((double) System.currentTimeMillis() / 1000);
	}
	
	public void teleopPeriodic() {
		//update IMU and time
		currAccel = - (imu.getAccelY() * 9.8) - initAccel; //acc in m/s^2
		currAngle = - (imu.getAngleZ() / 4) - initAngle; //scale is 4 units per degree
		currTime = ((double) System.currentTimeMillis() / 1000) - initTime;
		deltaTime = currTime - prevTime;
		
		//update velocity and position
		currVel = prevVel + currAccel * deltaTime;
		currX = prevX + currVel * Math.sin(currAngle) * deltaTime;
		currY = prevY + currVel * Math.cos(currAngle) * deltaTime;
		
		//display all values
		SmartDashboard.putNumber("Acceleration (N/kg)", currAccel); //line plot
		SmartDashboard.putNumber("Angle (degrees)", currAngle); //line plot
		SmartDashboard.putNumber("Time (sec)", currTime); //text box
		SmartDashboard.putNumber("Time per loop", deltaTime); //line plot
		SmartDashboard.putNumber("Velocity", currVel); //line plot
		SmartDashboard.putNumber("X Position", currX); //line plot
		SmartDashboard.putNumber("Y Position", currY); //line plot
		
		//move the robot
		drive.tankDrive(leftJoy.getY(), rightJoy.getY());
		
		//store the current values as previous for next iteration of loop
		prevVel = currVel;
		prevX = currX;
		prevY = currY;
		prevTime = currTime;
	}
}