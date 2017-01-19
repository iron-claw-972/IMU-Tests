
package org.usfirst.frc.team9972.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.*;

public class Robot extends IterativeRobot {
    ADIS16448_IMU imu;

    public Robot() {
    }
    
    public void robotInit() {
        imu = new ADIS16448_IMU();
    }

    public void autonomous() {
    }

    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
            SmartDashboard.putData("IMU", imu);
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

    public void test() {
    }
}
