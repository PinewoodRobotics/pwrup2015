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
	
	public final static double JoyKneeOne = 0.1;        // end of the deadzone & first knee of joystick range which starts 'maneuvering range'
    public final static double JoyKneeTwo = 0.8;         // second knee of joystick range which ends 'maneuvering range' and starts 'speed range'
    public final static double JoyMaxRange = 1.0;        // maximum input range of joysticks
	
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
    public void autonomousPeriodic() 
    {
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
    public void teleopInit()
    {
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() 
    {
    	
    	double Y = driver.getY();
    	double X = driver.getX();
    	double R = driver.getZ();    	

         if (Math.abs(Y) < JoyKneeOne) // deadzones
         {
             Y = 0.0;
         }

         if ((Math.abs(Y) >= JoyKneeOne) && (Math.abs(Y) <= JoyKneeTwo)) // mapping for maneuvering range
         {
             if (Y < 0.0)
             {
                 Y = (3.0 / 5.0) * Y - 0.02;     // changes raw negative input into a maneuverable speed
             } 
             else
             {
                 Y = (3.0 / 5.0) * Y + 0.02;     // changes raw positive input into a maneuverable speed
             }
         } 
         else
         {
             if((Math.abs(Y) > JoyKneeTwo) && (Math.abs(Y) <= JoyMaxRange)) // mapping for speed range
             {
                 if(Y < 0)
                 {
                     Y = (5.0 / 2.0) * Y + 1.5;  // changes raw negative input into a fast speed
                 } 
                 else
                 {
                     Y = (5.0 / 2.0) * Y - 1.5;  // changes raw positive input into a fast speed
                 }
             }
         }
         
         
         if(Math.abs(X) < JoyKneeOne) // deadzones
         {
             X = 0.0;
         }

         if((Math.abs(X) >= JoyKneeOne) && (Math.abs(X) <= JoyKneeTwo)) // mapping for maneuvering range
         {
             if(X < 0.0)
             {
                 X = (3.0 / 5.0) * X - 0.02;
             } 
             else
             {
                 X = (3.0 / 5.0) * X + 0.02;
             }
         } 
         else
         {
             if((Math.abs(X) > JoyKneeTwo) && (Math.abs(X) <= JoyMaxRange)) // mapping for speed range
             {
                 if (X < 0.0)
                 {
                     X = (5.0 / 2.0) * X + 1.5;
                 }
                 else
                 {
                     X = (5.0 / 2.0) * X - 1.5;
                 }
             }
         }
         
         if(Math.abs(R) < JoyKneeOne) // deadzones
         {
             R = 0.0;
         }

         if((Math.abs(R) >= JoyKneeOne) && (Math.abs(R) <= JoyKneeTwo)) //mapping for maneuvering range
         {
             if(R < 0.0)
             {
                 R = (3.0 / 5.0) * R - 0.02;     //changes raw negative input into a maneuverable speed
             } 
             else
             {
                 R = (3.0 / 5.0) * R + 0.02;     //changes raw positive input into a maneuverable speed
             }
         } 
         else
         {
             if((Math.abs(R) > JoyKneeTwo) && (Math.abs(R) <= JoyMaxRange)) //mapping for speed range
             {
                 if(R < 0)
                 {
                     R = (5.0 / 2.0) * R + 1.5;  //changes raw negative input into a fast speed
                 } 
                 else
                 {
                     R = (5.0 / 2.0) * R - 1.5;  //changes raw positive input into a fast speed
                 }
             }
         }


    	
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
    	
    	
    	
    	
    	// TODO: make another method for controlling and call it in teleop for organization
    		
        myRobot.arcadeDrive(stick);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
