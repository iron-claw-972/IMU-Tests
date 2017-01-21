package org.usfirst.frc.team9972.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.ADXL362;

public class Robot extends IterativeRobot {
	ADIS16448_IMU imu = new ADIS16448_IMU();

	Joystick leftJoy = new Joystick(0);
	Joystick rightJoy = new Joystick(1);
	Joystick operatorJoy = new Joystick(2);

	Victor backRightMotor = new Victor(0);
	Victor backLeftMotor = new Victor(1);
	Victor frontRightMotor = new Victor(2);
	Victor frontLeftMotor = new Victor(3);
	
	Accelerometer.Range range;
	ADXL362 accel = new ADXL362(Range.k4G);
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
//	Victor pidStraightValue = new Victor(10); //this is sketchy code, shouldn't actually run any motors
//	PIDController pidDriveStraight = new PIDController(0, 0, 0, gyro, pidStraightValue);
//	
//	Victor pidTurnValue = new Victor(11); //this is sketchy code, shouldn't actually run any motors //crashes program, robot's don't quit error
//	PIDController pidTurn = new PIDController(0, 0, 0, gyro, pidTurnValue);// robots don't quit
	
	double setAngle_Straight = 0.0;
	double setAngle_Turn = 0.0;
	
	//we also need to correct the angle output so that -360 = +360 and +360 = 0 but don't worry about it now
	
	double driveStraightSpeed = 0.0;

	double[] acceleration = new double[3];

	RobotDrive drive = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

	boolean leftPressedLastTime = false;
	boolean driveStraight = false;
	
	boolean rightPressedLastTime = false;
	boolean turnToPoint = false;
	


	public void teleopInit() {
		gyro.calibrate();
		imu.calibrate();
		/*
		 * Graph gets all messed up if we move our robot before the graph calibrates.
		 * We need to wait approx 3 seconds (not positive; it would be best to remeasure)
		 * before moving the robot so the graph has time to calibrate
		*/
		
	}

	public void autonomousPeriodic() {
		while((imu.getAngleX()/4)<180){
		  drive.tankDrive(-.5, .5);
		  SmartDashboard.putData("IMU", imu); 
		  SmartDashboard.putNumber("Angle Y", imu.getAngleY()/4);
		  SmartDashboard.putNumber("Angle X", imu.getAngleX()/4);
		  SmartDashboard.putNumber("Angle Z", imu.getAngleZ()/4);
		  SmartDashboard.putNumber("Accel X", imu.getAccelX());
		  SmartDashboard.putNumber("Accel Y", imu.getAccelY());
		  SmartDashboard.putNumber("Accel Z", imu.getAccelZ());
		}
	}
	
	public void teleopPeriodic() {
		/*
		double currentAngle = gyro.getAngle();
		boolean leftIsPressed = leftJoy.getRawButton(1);
		
		double kP = (((leftJoy.getZ() * -1) + 1) / 2.0) * 0.03;
		double kI = (((rightJoy.getZ() * -1) + 1) / 2.0) * 0.0008;
		double kD = (((operatorJoy.getThrottle() * -1) + 1) / 2.0) * 0.005;

		if (leftIsPressed && !leftPressedLastTime) {
			driveStraight = !driveStraight;
			setAngle_Straight = currentAngle;
			if(driveStraight) {
				pidDriveStraight.setPID(-0.018, -0.0008, 0.0002); //D is in the opposing direction of P and I
				pidDriveStraight.setSetpoint(setAngle_Straight);
				pidDriveStraight.enable();
			} else {
				pidDriveStraight.reset();
			}
		}
		leftPressedLastTime = leftIsPressed;
		

		boolean rightIsPressed = rightJoy.getRawButton(1);
		if (rightIsPressed && !rightPressedLastTime) {
			turnToPoint = !turnToPoint;
			setAngle_Turn = currentAngle + 90.0;
			if(turnToPoint) {
				pidTurn.setPID(-kP, -kI, kD);
				pidTurn.setSetpoint(setAngle_Turn);
				pidTurn.enable();
			} else {
				pidTurn.reset();
			}
		}
		rightPressedLastTime = rightIsPressed;
		
		if (driveStraight) {
			driveStraightSpeed = -leftJoy.getY(); //joysticks wrong sign
			drive.tankDrive(driveStraightSpeed - (1 - Math.abs(driveStraightSpeed)) * pidDriveStraight.get(), driveStraightSpeed + (1 - Math.abs(driveStraightSpeed)) * pidDriveStraight.get()); //this code should get the output of the pid and add it to the desired speed so that the robot moves at desired speed while using pid to adjust the angle (and never making speed values over +/-1)
		} else if (turnToPoint) {
			drive.tankDrive(-pidTurn.get(), pidTurn.get());
		} else {
			drive.tankDrive(-leftJoy.getY(), -rightJoy.getY()); //joysticks wrong sign
		}

		acceleration[0] = accel.getX();
		acceleration[1] = accel.getY();
		acceleration[2] = accel.getZ();
		SmartDashboard.putBoolean("driveStraight", driveStraight);
		SmartDashboard.putBoolean("turnToPoint", turnToPoint);
		SmartDashboard.putNumber("X Acceleration", acceleration[0]);
		SmartDashboard.putNumber("Y Acceleration", acceleration[1]);
		SmartDashboard.putNumber("Z Acceleration", acceleration[2]);
		SmartDashboard.putNumber("Angle Graph", currentAngle % 360);
		SmartDashboard.putNumber("Error Turn", -setAngle_Turn + currentAngle);
		SmartDashboard.putNumber("Current Angle Value", currentAngle); //to have both graph and value displayed simultaneously on Smart Dashboard
		SmartDashboard.putNumber("P * 10", kP * 10);
		SmartDashboard.putNumber("I * 100", kI*100);
		SmartDashboard.putNumber("D * 10", kD*10);*/
		
		
		while((imu.getAngleZ()/4)< 180){//need to make it so that it does 180 turn after a button price
			// need to save last angle and make it so that it turns until it becomes either Z initial position / 4 before +180 or -180
			  drive.tankDrive(.5, -.5);
			  SmartDashboard.putData("IMU", imu); 
			  SmartDashboard.putNumber("Angle Y", imu.getAngleY()/4);//For all angles, scale by dividing by 4 (ex: a 90 degree turn by the robot returns a Z angle change of 360)
			  SmartDashboard.putNumber("Angle X", imu.getAngleX()/4);
			  SmartDashboard.putNumber("Angle Z", imu.getAngleZ()/4);
			  SmartDashboard.putNumber("Accel X", imu.getAccelX());
			  SmartDashboard.putNumber("Accel Y", imu.getAccelY());
			  SmartDashboard.putNumber("Accel Z", imu.getAccelZ());
		}
	}
}