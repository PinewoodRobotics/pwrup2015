package org.usfirst.frc.team4765.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	int autoLoopCounter;
	public static Joystick driver = new Joystick(0);
	public static CANTalon motor1 = new CANTalon(1); 
	public static CANTalon motor2 = new CANTalon(2);
	public static CANTalon motor3 = new CANTalon(3);
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        System.out.println("robotInit()");
    	myRobot = new RobotDrive(0,1);
    	stick = new Joystick(0);
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
			autoLoopCounter++;
			} else {
			myRobot.drive(0.0, 0.0); 	// stop robot
		}
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	double Y = driver.getY();
    	double X = driver.getX();
    	double R = driver.getZ(); // this might be wrong 
    	
    	double motor1speed = X + Y + -0.5 * R;
    	double motor2speed = -1.0 * X + Y + 0.5 * R;
    	double motor3speed = 0.5 * X + R;
    	
    	double biggestValue = Math.max(motor1speed, Math.max(motor2speed, motor3speed));
    	
    	if(biggestValue > 1.0)
    	{
    		motor1speed /= biggestValue;
    		motor2speed /= biggestValue;
    		motor3speed /= biggestValue;
    	}
    		
    	motor1.set(-1.0 * motor1speed);
    	motor2.set(motor2speed);
    	motor3.set(motor3speed);
    	
    	
    	
    	
    	//make another method for controlling and call it in teleop for organization?
    		
        myRobot.arcadeDrive(stick);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
