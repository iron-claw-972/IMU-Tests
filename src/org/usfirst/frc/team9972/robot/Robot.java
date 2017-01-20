package org.usfirst.frc.team9972.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.analog.adis16448.frc.ADIS16448_IMU;

public class Robot extends IterativeRobot {
	ADIS16448_IMU imu = new ADIS16448_IMU();
	Victor frontLeftMotor = new Victor(0);
	Victor rearLeftMotor = new Victor(1);
	Victor frontRightMotor = new Victor(2);
	Victor rearRightMotor = new Victor(3);

	RobotDrive go = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
	
	
	
	public Robot() {
	}

	public void robotInit() {
		//imu.calibrate();
		frontRightMotor.setInverted(true);//Because kitbot victor wiring is screwed up 

	}

	public void autonomousPeriodic() {
		//	while(Timer.getMatchTime() < 3){
		//while((imu.getAngleX()/4)<180){
		  go.tankDrive(.4, .4);//slow but dangerous, fix
		  SmartDashboard.putData("IMU", imu); 
		  SmartDashboard.putNumber("Angle Y", imu.getAngleY()/4);
		  SmartDashboard.putNumber("Angle X", imu.getAngleX()/4);
		  SmartDashboard.putNumber("Angle Z", imu.getAngleZ()/4);
		  SmartDashboard.putNumber("Accel X", imu.getAccelX());
		  SmartDashboard.putNumber("Accel Y", imu.getAccelY());
		  SmartDashboard.putNumber("Accel Z", imu.getAccelZ());
	//		}
	}

	public void TeleOperated() {
		
	}

	public void test() {
	}
}
