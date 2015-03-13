
package org.usfirst.frc.team687.robot;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	Joystick leftJoy, rightJoy, articJoy;
	VictorSP ftLeft, ftRight, bkLeft, bkRight;
	IMUAdvanced imu;
	boolean first_iteration = true;
	CANTalon artic;
	Encoder encoder;
	NerdyAutonomous auto;
	PowerDistributionPanel pdp;
	
    public void robotInit() {
    	leftJoy = new Joystick(0);
    	rightJoy = new Joystick(1);
    	articJoy = new Joystick(2);
    	ftLeft = new VictorSP(2);
    	ftRight = new VictorSP(3);
    	bkLeft = new VictorSP(4);
    	bkRight = new VictorSP(5);
    	imu = new IMUAdvanced(new SerialPort(57600,SerialPort.Port.kMXP));
    	artic = new CANTalon(1);
    	encoder = new Encoder(9,8);
    
    }

    /**
     * This function is called periodically during autonomous
     */
    public void disabledPeriodic() {
    	boolean is_calibrating = imu.isCalibrating();
        if(first_iteration && is_calibrating)   {
            Timer.delay(0.3);
            imu.zeroYaw();
            first_iteration = false;
        }
    }

    Timer t;
    
    public void autonomousInit()	{
    	t = new Timer();
    	t.reset();
    	t.start();
    }
    
    public void autonomousPeriodic()	{
    	SmartDashboard.putNumber("AutonomousTime", t.get());
    	if(t.get()<2)	{
    		ftLeft.set(.5);
    		ftRight.set(-.5);
    		bkLeft.set(.5);
    		bkRight.set(-.5);
    	}	else	{
    		ftLeft.set(threshold(2.5-t.get()));
    		ftRight.set(-threshold(2.5-t.get()));
    		bkLeft.set(threshold(2.5-t.get()));
    		bkRight.set(-threshold(2.5-t.get()));
    	}
    	SmartDashboard.putNumber("Artic Current", artic.getOutputCurrent());
    	SmartDashboard.putNumber("Artic Voltage", artic.getOutputVoltage());
    }
    
    public double threshold(double d)	{
    	if(d>1)	{
    		return 1;
    	}	else if(d<0)	{
    		return 0;
    	}	else	{
    		return d;
    	}
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	boolean is_calibrating = imu.isCalibrating();
        if(first_iteration && is_calibrating)   {
            Timer.delay(0.3);
            imu.zeroYaw();
            first_iteration = false;
        }
    	NerdyDrive.setHeader(-imu.getYaw());
    	SmartDashboard.putNumber("yaw",imu.getYaw());
        boolean beta = rightJoy.getRawButton(4);
        double fl, fr, bl, br;
        if(beta)	{
        	NerdyDrive.driveBeta(leftJoy.getX(), -leftJoy.getY(), rightJoy.getX(), -rightJoy.getY());
        }	else	{
        	NerdyDrive.driveAlpha(leftJoy.getX(), -leftJoy.getY(), rightJoy.getX());
        }
        
        if(leftJoy.getRawButton(4)){
        	imu.zeroYaw();
        }
        
        fl = NerdyDrive.getFtLeft();
        fr = NerdyDrive.getFtRight();
        bl = NerdyDrive.getBkLeft();
        br = NerdyDrive.getBkRight();
        
        ftLeft.set(fl);
        ftRight.set(fr);
        bkLeft.set(bl);
        bkRight.set(br);
        
        double articPower = articJoy.getY();
        artic.set(articPower);
        SmartDashboard.putNumber("Artic Current", artic.getOutputCurrent());
        SmartDashboard.putNumber("Artic Voltage", artic.getOutputVoltage());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
