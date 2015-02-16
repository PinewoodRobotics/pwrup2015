package org.usfirst.frc.team4765.robot;

import org.usfirst.frc.team4765.robot.PIDTower;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.can.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

/* WPILibJ doc: <http://first.wpi.edu/FRC/roborio/release/docs/java/index.html> */

/**
 * @author Pavel Khokhlov
 * @author Dean Reece
 * 
 * @version 14 February 2015
 */

public class Robot extends IterativeRobot // check the error, this happened after our late night drawing trouble shooting
{
	int autoLoopCounter;

	public static CANTalon motor1 = new CANTalon(1); 
	public static CANTalon motor2 = new CANTalon(2); // motors for driving
	public static CANTalon motor3 = new CANTalon(3); 
	 
	//public static Talon talon1 = new Talon(8);
	//public static Talon talon2 = new Talon(9); // motors for the chain
	
	public static Joystick driver   = new Joystick(0); // joystick that controls the driving
	JoystickButton trigger = new JoystickButton(driver, 1);
	JoystickButton refreshPrefs = new JoystickButton(driver, 8);
	JoystickButton run  = new JoystickButton(driver, 11);
	JoystickButton step = new JoystickButton(driver, 12);		//joystick values
	JoystickButton raiseStory = new JoystickButton(driver, 6);
	JoystickButton lowerStory = new JoystickButton(driver, 4);
	JoystickButton raiseElevation = new JoystickButton(driver, 3);	// down
	JoystickButton lowerElevation = new JoystickButton(driver, 5); 	// up
	
	static DigitalInput heightLimit = new DigitalInput(7);
	static DigitalInput tower1TotePresent = new DigitalInput(1);
	static DigitalInput tower2TotePresent = new DigitalInput(2);
	
	public static PIDTower PIDTower1 = new PIDTower(8, 8, 3, 4);
	public static PIDTower PIDTower2 = new PIDTower(9, 9, 5, 6);
	
	public static final double DeadZone     = 0.05;
	public static final double JoyKneeOneX_ = 0.1;        // end of the deadzone & first knee of joystick range which starts 'maneuvering range'
    public static final double JoyKneeTwoX_ = 0.8;        // second knee of joystick range which ends 'maneuvering range' and starts 'speed range'
    public static final double JoyMaxRange_ = 1.0;        // maximum input range of joysticks
    public static final double JoyKneeOneY_ = 0;		  // starts the first leg of the mapping
    public static final double JoyKneeTwoY_ = 0.35;		  
    
    Preferences prefs = Preferences.getInstance();
    
    double TowerP;
    double TowerI;
    double TowerD;
    public static double towerMin;
    public static double towerMax;
    
    double P;				// PID loop values
    double I;				
    double D;
    double F;
    int    iZone; 			// i-zone that gives I a limit to cumulation
    double Ramp; 			// closeLoopRampRate Maximum change in voltage, Unit: volts/sec
    double MaxRPM;
    double StartPosition;
    int Profile;	    // value of 0 or 1

    double[] motorSpeed = new double[4]; //holds motor speeds (in rpm)
    
    int CANTimeouts;
    
    public static boolean prevRefreshPressed        = false;	// remembers when the buttons on the joysitck were last pressed
    public static boolean lastTrigger               = false;
    public static boolean prevRaiseStoryPressed     = false;
    public static boolean prevLowerStoryPressed     = false;
    public static boolean prevRaiseElevationPressed = false;
    public static boolean prevLowerElevationPressed = false;
    
    public void CANTimeout()
    {
        CANTimeouts++;
        SmartDashboard.putNumber("CANTimeouts", CANTimeouts);
    }
	
    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     * TODO: implement a timeout exception handler
     */
    public void robotInit() 
    {
    	motor1.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	motor2.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	motor3.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	
    	updatePrefs();
    	
    	System.out.println("### GOT EM' COACH  ###");
    }    
    
    /**
     * This function is called once before autonomous control
     */
    public void disabledInit()
    {
        System.out.println("### DISABLED ###");
    }

    
    /**
     * Prints and sets PID values on the smardashboard for CANTalons and PIDTowers.
     * 
     */
    public void updatePrefs()
    {
        MaxRPM = prefs.getDouble("MaxRPM", 862.0);
        P = prefs.getDouble("P", 0.0);   // can change values from here, press button to activate changes        
        I = prefs.getDouble("I", 0.0);
        D = prefs.getDouble("D", 0.0);
        F = prefs.getDouble("F", 1.0);
        iZone = prefs.getInt("iZone", 0);
        Ramp = prefs.getDouble("Ramp", 1200.0);
        
        SmartDashboard.putNumber("CANTalon P", P);  //displays PID values on SmartDash
        SmartDashboard.putNumber("CANTalon I", I);
        SmartDashboard.putNumber("CANTalon D", D);
        SmartDashboard.putNumber("CANTalon F", F);
        SmartDashboard.putNumber("CANTalon iZone", iZone);
        SmartDashboard.putNumber("CANTalon Ramp", Ramp);
        SmartDashboard.putNumber("MaxRPM", MaxRPM);
        
        // PIDTOWER
        
        TowerP = prefs.getDouble("TowerP", 0.004);   // can change values from here, press button to activate changes        
        TowerI = prefs.getDouble("TowerI", 0.0);
        TowerD = prefs.getDouble("TowerD", 0.0);
        towerMin = prefs.getDouble("TowerMin", -0.5);
        towerMax = prefs.getDouble("TowerMax", 0.5);	
        PIDTower1.offSet_ = prefs.getDouble("Tower1 Offset", 0.0);
        PIDTower2.offSet_ = prefs.getDouble("Tower2 Offset", 0.0);
        
		SmartDashboard.putNumber("TowerMin", towerMin);
		SmartDashboard.putNumber("TowerMax", towerMax);
        
        try
        {
            motor1.setPID(P, I, D, F, iZone, Ramp, 0);  //sets PID constants 
            motor2.setPID(P, I, D, F, iZone, Ramp, 0);
            motor3.setPID(P, I, D, F, iZone, Ramp, 0);
            
            motor1.changeControlMode(CANTalon.ControlMode.Speed);
        	motor2.changeControlMode(CANTalon.ControlMode.Speed);
        	motor3.changeControlMode(CANTalon.ControlMode.Speed);
            
            motor1.enableControl(); //starts feedback ctrl
            motor2.enableControl();
            motor3.enableControl();
            
            PIDTower1.controller_.setPID(TowerP, TowerI, TowerD);
            PIDTower2.controller_.setPID(TowerP, TowerI, TowerD);
            
            PIDTower1.controller_.setOutputRange(towerMin, towerMax);
            PIDTower2.controller_.setOutputRange(towerMin, towerMax);
            
            PIDTower1.controller_.enable();
            PIDTower2.controller_.enable();
            
        	PIDTower1.goHome();
        	PIDTower2.goHome();
        } 
        catch (CANInvalidBufferException ex)
        {
            CANTimeout();
        }
        System.out.println("Finished prefs.");
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() 
    {
    	autoLoopCounter = 0;
    }
   
    /**
     * This function is called periodically during autonomous.
     */
    public void autonomousPeriodic() 
    {
    	/*
    	if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
			autoLoopCounter++;
			} else {
			myRobot.drive(0.0, 0.0); 	// stop robot
		}
		*/
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode.
     */
    public void teleopInit()
    {
    	
    }
        
    /**
     * This takes the raw input values from the joystick and maps them into more convenient speeds.
     * The pre-set values can be changed where constants are initialized.
     */    
    public double mapDrivingValue(double V)
    {
    	double m1 = (JoyKneeTwoY_)/(JoyKneeTwoX_ - DeadZone);
    	double m2 = (JoyMaxRange_ - JoyKneeTwoY_)/(JoyMaxRange_ - JoyKneeTwoX_);
    	
    	if(Math.abs(V) < JoyKneeOneX_) // deadzone
    	{
    		V = 0.0;
    	}
    	else
    	{
	    	if ((Math.abs(V) >= JoyKneeOneX_) && (Math.abs(V) <= JoyKneeTwoX_)) // mapping for maneuvering range
	    	{
	    		if( V < 0.0)
	    		{
	    			V = m1 * (V + JoyKneeTwoX_) - JoyKneeTwoY_;
	    		}
	    		else
	    		{
	    			V = m1 * (V - JoyKneeTwoX_) + JoyKneeTwoY_;
	    		}
	    	}
	    	else
	    	{
	    		if((Math.abs(V) > JoyKneeTwoX_) && (Math.abs(V) <= JoyMaxRange_)) // mapping for speed range
	            {
	                if(V < 0.0)
	                {
	                    V = m2 * (V + JoyMaxRange_) - JoyMaxRange_;  // changes raw negative input into a fast speed
	                } 
	                else
	                {
	                	V = m2 * (V - JoyMaxRange_) + JoyMaxRange_;  // changes raw positive input into a fast speed
	                }
	            }
	    	}
    	}
    	return V;
    }
    
    /**
     * This function is called periodically during operator control.
     * This function is responsible for the driving and the chain mechanism.
     */
    public void teleopPeriodic() 
    {
    	periodic();
    	
    	boolean refreshPressed = refreshPrefs.get();

    	if(refreshPressed && (prevRefreshPressed == false))
    	{
    		updatePrefs();
    		//tower1.stop();
    		//tower2.stop();
    	}
    	prevRefreshPressed = refreshPressed;

    	if(PIDTower1.readyForCommand() && PIDTower2.readyForCommand())
    	{
	    	// going up story
			boolean raiseStoryPressed = raiseStory.get();				// reads button values
	    	if(raiseStoryPressed && (prevRaiseStoryPressed == false))
	    	{
	    		
	    		PIDTower1.goUpStory();
	    		PIDTower2.goUpStory();
	    	}
	    	prevRaiseStoryPressed = raiseStoryPressed;
	
	    	// going down story
	    	boolean lowerStoryPressed = lowerStory.get();
	    	if(lowerStoryPressed && (prevLowerStoryPressed == false))
	    	{
	    		PIDTower1.goDownStory();
	    		PIDTower2.goDownStory();
	    	}
	    	prevLowerStoryPressed = lowerStoryPressed;
	    	
	    	// going up elevation
	    	boolean raiseElevationPressed = raiseElevation.get();
	    	if(raiseElevationPressed && (prevRaiseElevationPressed == false))
	    	{
	    		PIDTower1.setElevationState(true);
	    		PIDTower2.setElevationState(true);
	    	}
	    	prevRaiseElevationPressed = raiseElevationPressed;
	    	
	    	// going down elevation
	    	boolean lowerElevationPressed = lowerElevation.get();
	    	if(lowerElevationPressed && (prevLowerElevationPressed == false))
	    	{
	    		PIDTower1.setElevationState(false);
	    		PIDTower2.setElevationState(false);
	    	}
	    	prevLowerElevationPressed = lowerElevationPressed;
    	}
    	double Y = driver.getY();
    	double X = driver.getX();
    	double R = driver.getZ(); 

        Y = mapDrivingValue(Y);
        X = mapDrivingValue(X);		// changes the values for easier driving
        R = mapDrivingValue(R);
    	
    	double motor1speed = X - Y + -0.5 * R; 
    	double motor2speed = - X - Y + 0.5 * R;
    	double motor3speed = 0.5 * X + R;
    	
    	double biggestValue = Math.max(motor1speed, Math.max(motor2speed, motor3speed));
    	
    	if(biggestValue > 1.0)
    	{
    		motor1speed /= biggestValue;
    		motor2speed /= biggestValue;
    		motor3speed /= biggestValue;
    	}
    	
    	double throttle = (driver.getThrottle() + 1) / 2;
    	
    	motor1speed = motor1speed * throttle;
    	motor2speed = motor2speed * throttle;
    	motor3speed = motor3speed * throttle;
    		
    	motor1.set(MaxRPM * motor1speed * -1.0);
    	motor2.set(MaxRPM * motor2speed);				// has to be in velocity mode
    	motor3.set(MaxRPM * motor3speed);
    }
    
    /**
     * Launched in every periodic function. 
     */
    public void periodic()
    {
    	PIDTower1.periodic();
    	PIDTower2.periodic();
    	PIDTower1.setHeightLimit(!heightLimit.get());
    	PIDTower2.setHeightLimit(!heightLimit.get());
    	SmartDashboard.putBoolean("Elevation State1", PIDTower1.getElevationState());
    	SmartDashboard.putBoolean("Elevation State2", PIDTower2.getElevationState());
    	SmartDashboard.putBoolean("Tower1 Ready", PIDTower1.readyForCommand());
    	SmartDashboard.putBoolean("Tower2 Ready", PIDTower2.readyForCommand());
    	SmartDashboard.putNumber("Tower Encoder 1", PIDTower1.encoder_.pidGet());
    	SmartDashboard.putNumber("Tower Encoder 2", PIDTower2.encoder_.pidGet());
    	SmartDashboard.putBoolean("Tower1 button", !tower1TotePresent.get());
    	SmartDashboard.putBoolean("Tower2 button", !tower2TotePresent.get());
    	SmartDashboard.putBoolean("HallEffect1", PIDTower1.hallEffect_.get());
    	SmartDashboard.putBoolean("HallEffect2", PIDTower2.hallEffect_.get());
    	SmartDashboard.putBoolean("Homing1", PIDTower1.homing);
    	SmartDashboard.putBoolean("Homing2", PIDTower2.homing);
       // System.out.println(motor1.getEncVelocity() + "     " + motor2.getEncVelocity() + "     " + motor3.getEncVelocity());
    }
    
    /**
     * Runs the motors at max positive and negative speed and prints out the values.
     */
    public void testPeriodic3()
    { 
    	/*
    	MAX: 1283       926      867
    	MIN: -1287     -909     -874
    	MAX: 1279       927      869
    	MIN: -1276     -910     -874
    	MAX: 1272       931      873
    	MIN: -1268     -915     -874
    	MAX: 1272       928      872
    	MIN: -1265     -916     -875
    	MAX: 1270       928      870
    	MIN: -1265     -917     -876
        */
    	
    	motor1.set(1);
    	motor2.set(1);
    	motor3.set(1);
    	Timer.delay(0.5);
    	System.out.println("MAX: " + motor1.getEncVelocity() + "     " + motor2.getEncVelocity() + "     " + motor3.getEncVelocity());
    	motor1.set(0);
    	motor2.set(0);
    	motor3.set(0);
    	Timer.delay(0.5);
    	motor1.set(-1);
    	motor2.set(-1);
    	motor3.set(-1);
    	Timer.delay(0.5);
    	System.out.println("MIN: " + motor1.getEncVelocity() + "     " + motor2.getEncVelocity() + "     " + motor3.getEncVelocity());     	
    	motor1.set(0);
    	motor2.set(0);
    	motor3.set(0);
    	Timer.delay(0.5);
    }
    
    /**
     * Runs the motor at max speed and prints the value.
     * Used for tweaking PID values.
     */
    public void testPeriodic2()
    {
    	SmartDashboard.putNumber("Motor1Speed", motor1.getEncVelocity());
    	System.out.println(motor1.getEncVelocity());
    }
    
    /**
     * gets rid of the popup that shows the talons and doesn't let us see PID
     */
    public void testInit()
    {
    	LiveWindow.setEnabled(false);
    }
    
    /**
     * Prints halifax values.
     */
    public void printSensorValues()
    {
    	//System.out.println(hallEffect1.get() + "                    " + hallEffect2.get());
    }
}
