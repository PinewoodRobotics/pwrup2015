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
 * @author Pavel Khokhlov pkhokhlov@hotmail.com
 * @author Dean Reece	  dean@reeceweb.com
 * 
 * @version 17 February 2015
 */

public class Robot extends IterativeRobot // check the error, this happened after our late night drawing trouble shooting
{
	int autoLoopCounter;
	// Initializes CANTalons
	public static CANTalon motor1  = new CANTalon(1); 
	public static CANTalon motor2  = new CANTalon(2); // motors for driving
	public static CANTalon motor3  = new CANTalon(3); 
	// Initializes Joysticks for the driver & operator
	public static Joystick driver   = new Joystick(0); // joystick that controls the driving
	public static Joystick operator = new Joystick(1);
	// Initializes all buttons for the driver
	JoystickButton trigger         = new JoystickButton(driver, 1);		//joystick values
	JoystickButton refreshPrefs1   = new JoystickButton(driver, 8);		
	JoystickButton raiseStory1     = new JoystickButton(driver, 6);	
	JoystickButton lowerStory1     = new JoystickButton(driver, 4);
	JoystickButton raiseElevation1 = new JoystickButton(driver, 5);	// down
	JoystickButton lowerElevation1 = new JoystickButton(driver, 3); 	// up
	// Initializes all buttons for the operator
	JoystickButton refreshPrefs2   = new JoystickButton(operator, 8);		
	JoystickButton raiseStory2     = new JoystickButton(operator, 6);
	JoystickButton lowerStory2     = new JoystickButton(operator, 4);
	JoystickButton raiseElevation2 = new JoystickButton(operator, 5);	// down
	JoystickButton lowerElevation2 = new JoystickButton(operator, 3);	// up
	// our sensors for the heightlimit and tote detectors
	static DigitalInput heightLimit = new DigitalInput(7);
	static DigitalInput tower1TotePresent = new DigitalInput(1);
	static DigitalInput tower2TotePresent = new DigitalInput(2);
	// Initializes PIDTowers
	public static PIDTower PIDTower1 = new PIDTower(8, 8, 3, 4);
	public static PIDTower PIDTower2 = new PIDTower(9, 9, 5, 6);
	// Mapping values
	public static final double DeadZone     = 0.05;
	public static final double JoyKneeOneX_ = 0.1;        // end of the deadzone & first knee of joystick range which starts 'maneuvering range'
    public static final double JoyKneeTwoX_ = 0.8;        // second knee of joystick range which ends 'maneuvering range' and starts 'speed range'
    public static final double JoyMaxRange_ = 1.0;        // maximum input range of joysticks
    public static final double JoyKneeOneY_ = 0;		  // starts the first leg of the mapping
    public static final double JoyKneeTwoY_ = 0.35;		  
    // Preferences
    Preferences prefs = Preferences.getInstance();
    // PIDTOWER PID loop values
    double TowerP;
    double TowerI;
    double TowerD;
    public static double TowerMin;
    public static double TowerMax;
    // MOTOR PID loop values
    double P;				
    double I;				
    double D;
    double F;
    int    iZone; 			// i-zone that gives I a limit to cumulation
    double Ramp; 			// closeLoopRampRate Maximum change in voltage, Unit: volts/sec
    double MaxRPM;
    double StartPosition;
    int Profile;	    // value of 0 or 1
    int autonSetting;
    double distanceToTravel;
    double crabTrim;

    double[] motorSpeed = new double[4]; //holds motor speeds (in rpm)
    
    int CANTimeouts;
    
    public static boolean prevRefreshPressed        = false;	// remembers when the buttons on the joysitck were last pressed
    public static boolean lastTrigger               = false;
    public static boolean prevRaiseStoryPressed     = false;
    public static boolean prevLowerStoryPressed     = false;
    public static boolean prevRaiseElevationPressed = false;
    public static boolean prevLowerElevationPressed = false;
    public static boolean heightLimitState			= false;
    
    /**
     * add preference for which auton code we use - Done
     * auton1: home & stand still - Done
     * auton2: home & move forward - Done
     * TODO: auton3: home & pickuptote and move forward
     */
    
    public void CANTimeout()
    {
        CANTimeouts++;
        SmartDashboard.putNumber("CANTimeouts", CANTimeouts);
    }
	
    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
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
    	// Prefs for the CANTalon PID values
        MaxRPM = prefs.getDouble("MaxRPM", 862.0);
        P = prefs.getDouble("P", 0.0);        
        I = prefs.getDouble("I", 0.0);
        D = prefs.getDouble("D", 0.0);
        F = prefs.getDouble("F", 1.0);
        iZone = prefs.getInt("iZone", 0);
        Ramp = prefs.getDouble("Ramp", 1200.0);
        // Auton Preferences
        autonSetting = prefs.getInt("AutonSetting", 1);	// goes from 1 to 4
        distanceToTravel = prefs.getDouble("distance", 10.0);
        crabTrim = prefs.getDouble("CrabTrim", 0.75);	// manipulates the crabbing of motor3
        //displays PID values on SmartDash
        SmartDashboard.putNumber("CANTalon P", P);  
        SmartDashboard.putNumber("CANTalon I", I);
        SmartDashboard.putNumber("CANTalon D", D);
        SmartDashboard.putNumber("CANTalon F", F);
        SmartDashboard.putNumber("CANTalon iZone", iZone);
        SmartDashboard.putNumber("CANTalon Ramp", Ramp);
        SmartDashboard.putNumber("MaxRPM", MaxRPM);
        SmartDashboard.putNumber("Distance to Travel", distanceToTravel);
        SmartDashboard.putNumber("Auton setting", autonSetting);
        SmartDashboard.putNumber("Crab Trim", crabTrim);
        // PIDTOWER
        TowerP = prefs.getDouble("TowerP", 0.004);   // can change values from here, press button to activate changes        
        TowerI = prefs.getDouble("TowerI", 0.0);
        TowerD = prefs.getDouble("TowerD", 0.0);
        TowerMin = prefs.getDouble("TowerMin", -0.5);
        TowerMax = prefs.getDouble("TowerMax", 0.5);	
        PIDTower1.offSet_ = prefs.getDouble("Tower1Offset", 0.0);
        PIDTower2.offSet_ = prefs.getDouble("Tower2Offset", 0.0);
		SmartDashboard.putNumber("TowerMin", TowerMin);
		SmartDashboard.putNumber("TowerMax", TowerMax);
        
        try
        {
        	// sets PID constants 
            motor1.setPID(P, I, D, F, iZone, Ramp, 0);   
            motor2.setPID(P, I, D, F, iZone, Ramp, 0);
            motor3.setPID(P, I, D, F, iZone, Ramp, 0);
            // changes controlmode to speed for PID
            motor1.changeControlMode(CANTalon.ControlMode.Speed);
        	motor2.changeControlMode(CANTalon.ControlMode.Speed);
        	motor3.changeControlMode(CANTalon.ControlMode.Speed);
            // enables PID control
            motor1.enableControl(); //starts feedback ctrl
            motor2.enableControl();
            motor3.enableControl();
            // PIDTower PID values set
            PIDTower1.controller_.setPID(TowerP, TowerI, TowerD);
            PIDTower2.controller_.setPID(TowerP, TowerI, TowerD);
            // Sets maximum power for PIDTowers
            PIDTower1.controller_.setOutputRange(TowerMin, TowerMax);
            PIDTower2.controller_.setOutputRange(TowerMin, TowerMax);
            // enables PID control for PIDTowers
            PIDTower1.controller_.enable();
            PIDTower2.controller_.enable();
            // synchronizes PIDTowers
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
		auton1();
    	switch(autonSetting)
    	{
	    	case 1:
	    		break;
	    	
	    	case 2:
	    		auton2Init();
	    	break;
	    	
	    	case 3:
	    		break;
    	}
    }
   
    /**
     * This function is called periodically during autonomous.
     * This function selects which autonimous to run based on a preference: autonSetting.
     */
    public void autonomousPeriodic() 
    {
    	switch(autonSetting)
    	{
	    	case 1:
	    		break;
	    	
	    	case 2:
	    		auton2Periodic();
	    	break;
    	}
    	periodic();
    }
    
    /**
     * This function is called and homes the sensors and doesn't move. 
     */
    public void auton1()
    {
    	PIDTower1.goHome();
    	PIDTower2.goHome();
    	PIDTower1.periodic();
    	PIDTower2.periodic();
    }
    
    /**
     * This function moves the robot in auton based on a timer.
     */
    public void auton2Timer()
    {
    	Timer autonTimer = new Timer();
        autonTimer.reset();
        autonTimer.start();
        while(autonTimer.get() < 0.75)
        {
        	motor1.set(MaxRPM * -0.5);
        	motor2.set(MaxRPM *  0.5);
        }
        autonTimer.stop();
    }
    
    /**
     * This function moves the robot in auton based on encoders.
     * This is called in autonPeriodic
     */
    public void auton2Init()
    {
    	StartPosition = motor1.getPosition();
    	driveMath(0, -0.5, 0.04, 1);
    }
    
    public void auton3Init()
    {
        	StartPosition = motor1.getPosition();
        	driveMath(0, 0.5, 0, 1);
    }
    
    /**
     * This function
     */
    public void auton2Periodic()
    {
    	double distance = Math.abs(motor1.getPosition()-StartPosition);
    	if(distance > distanceToTravel)
    		driveMath(0, 0, 0, 0);
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
    	
    	boolean refreshPressed = isRefreshPressed();
    	if(refreshPressed && (prevRefreshPressed == false))
    	{
    		updatePrefs();
    	}
    	prevRefreshPressed = refreshPressed;

    	if(PIDTower1.readyForCommand() && PIDTower2.readyForCommand())
    	{
	    	// going up story
			boolean raiseStoryPressed = isRaiseStoryPressed();				// reads button values
	    	if(raiseStoryPressed && (prevRaiseStoryPressed == false))
	    	{
	    		
	    		PIDTower1.goUpStory();
	    		PIDTower2.goUpStory();
	    	}
	    	prevRaiseStoryPressed = raiseStoryPressed;
	    	// going down story
	    	boolean lowerStoryPressed = isLowerStoryPressed();
	    	if(lowerStoryPressed && (prevLowerStoryPressed == false))
	    	{
	    		PIDTower1.setHeightLimit(false);
	    		PIDTower2.setHeightLimit(false);
	    		PIDTower1.goDownStory();
	    		PIDTower2.goDownStory();
	    	}
	    	prevLowerStoryPressed = lowerStoryPressed;
	    	// going up elevation
	    	boolean raiseElevationPressed = isRaiseElevationPressed();
	    	if(raiseElevationPressed && (prevRaiseElevationPressed == false))
	    	{
	    		PIDTower1.setElevationState(true);
	    		PIDTower2.setElevationState(true);
	    	}
	    	prevRaiseElevationPressed = raiseElevationPressed;
	    	// going down elevation
	    	boolean lowerElevationPressed = isLowerElevationPressed();
	    	if(lowerElevationPressed && (prevLowerElevationPressed == false))
	    	{
	    		PIDTower1.setHeightLimit(false);
	    		PIDTower2.setHeightLimit(false);
	    		PIDTower1.setElevationState(false);
	    		PIDTower2.setElevationState(false);
	    	}
	    	prevLowerElevationPressed = lowerElevationPressed;
    	}
    	// if our towers are ready, both buttons for tote detection are pressed, and the trigger are pressed, the robot automatically picks up a tote
    	if(PIDTower1.readyForCommand() && PIDTower2.readyForCommand() && !tower1TotePresent.get() && !tower2TotePresent.get() && trigger.get())
    	{
    		PIDTower1.goUpStory();
    		PIDTower2.goUpStory();
    	}
    	// gets the values for X, Y, and R from the driver
    	double Y = driver.getY();
    	double X = driver.getX();
    	double R = driver.getZ(); 
    	// changes the values for easier driving
        Y = mapDrivingValue(Y);
        X = mapDrivingValue(X);		
        R = mapDrivingValue(R);
        
        double throttle = (driver.getThrottle() + 1) / 2;
        
        driveMath(X, Y, R, throttle);
    }
    
    /**
     * 	This function takes the values from the joystick and gives the commands to the motors
     */
    public void driveMath(double X, double Y, double R, double throttle)
    {   
    	if(X < -0.5)
    		X = -0.5;
    	
    	double motor1speed = X - Y + -0.5 * R; 
    	double motor2speed = - X - Y + 0.5 * R;
    	double motor3speed = crabTrim * X + R;
        
    	double biggestValue = Math.max(motor1speed, Math.max(motor2speed, motor3speed));
    	
    	if(biggestValue > 1.0)
    	{
    		motor1speed /= biggestValue;
    		motor2speed /= biggestValue;
    		motor3speed /= biggestValue;
    	}
   
    	motor1speed = motor1speed * throttle;
    	motor2speed = motor2speed * throttle;
    	motor3speed = motor3speed * throttle;
    	
    	motor1.set(MaxRPM * motor1speed * -1.0);
    	motor2.set(MaxRPM * motor2speed);				// has to be in velocity mode
    	motor3.set(MaxRPM * motor3speed);
    }
    
    /**
     * This function updates our booleans for our sensors and displays them on our SmartDashboard.
     * This function is responsible for the heightLimit logic.
     * Launched in every periodic function. 
     */
    public void periodic()
    {
    	PIDTower1.periodic();
    	PIDTower2.periodic();
    	if(!heightLimit.get() == true)
    	{
    		PIDTower1.setHeightLimit(true);
    		PIDTower2.setHeightLimit(true);
    	}
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
     * All isXXXXXXPressed() functions allow for both joysticks to control the towers meaning driver & operator control the towers
     */
    public boolean isRefreshPressed()
    {
    	boolean refreshPressed1 = refreshPrefs1.get();
    	boolean refreshPressed2 = refreshPrefs2.get();
    	if(refreshPressed1 || refreshPressed2)
    	{
    		return true;
    	}
    	
    	return false;
    }
    
    public boolean isRaiseStoryPressed()
    {
    	boolean raiseStoryPressed1 = raiseStory1.get();
    	boolean raiseStoryPressed2 = raiseStory2.get();
    	if(raiseStoryPressed1 || raiseStoryPressed2)
    	{
    		return true;
    	}
    	
    	return false;
    }
    
    public boolean isLowerStoryPressed()
    {
    	boolean lowerStoryPressed1 = lowerStory1.get();
    	boolean lowerStoryPressed2 = lowerStory2.get();
    	if(lowerStoryPressed1 || lowerStoryPressed2)
    	{
    		return true;
    	}
    	
    	return false;
    }
    
    public boolean isRaiseElevationPressed()
    {
    	boolean raiseElevationPressed1 = raiseElevation1.get();
    	boolean raiseElevationPressed2 = raiseElevation2.get();
    	if(raiseElevationPressed1 || raiseElevationPressed2)
    	{
    		return true;
    	}
    	
    	return false;
    }
    
    public boolean isLowerElevationPressed()
    {
    	boolean lowerElevationPressed1 = lowerElevation1.get();
    	boolean lowerElevationPressed2 = lowerElevation2.get();
    	if(lowerElevationPressed1 || lowerElevationPressed2)
    	{
    		return true;
    	}
    	
    	return false;
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
