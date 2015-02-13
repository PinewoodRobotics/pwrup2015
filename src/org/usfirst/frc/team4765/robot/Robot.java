package org.usfirst.frc.team4765.robot;

import org.usfirst.frc.team4765.robot.Tower.State;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
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
 * @version 12 February 2015
 */

public class Robot extends IterativeRobot // check the error, this happened after our late night drawing trouble shooting
{
	int autoLoopCounter;

	public static CANTalon motor1 = new CANTalon(1); 
	public static CANTalon motor2 = new CANTalon(2); // motors for driving
	public static CANTalon motor3 = new CANTalon(3); 
	 
	public static Talon talon1 = new Talon(8);
	public static Talon talon2 = new Talon(9); // motors for the chain
	
	public static Joystick driver   = new Joystick(0); // joystick that controls the driving
	JoystickButton trigger = new JoystickButton(driver, 1);
	JoystickButton refreshPrefs = new JoystickButton(driver, 8);
	JoystickButton run  = new JoystickButton(driver, 11);
	JoystickButton step = new JoystickButton(driver, 12);		//joystick values
	JoystickButton raise = new JoystickButton(driver, 6);
	JoystickButton lower = new JoystickButton(driver, 4);
	
	static DigitalInput heightLimit = new DigitalInput(7);
	static DigitalInput hallEffect1 = new DigitalInput(8);
	static DigitalInput hallEffect2 = new DigitalInput(9);
	
	public static Tower tower1 = new Tower(talon1, hallEffect1, heightLimit);
	public static Tower tower2 = new Tower(talon2, hallEffect2, heightLimit);
	
	public final static double DeadZone     = 0.05;
	public final static double JoyKneeOneX_ = 0.1;        // end of the deadzone & first knee of joystick range which starts 'maneuvering range'
    public final static double JoyKneeTwoX_ = 0.8;        // second knee of joystick range which ends 'maneuvering range' and starts 'speed range'
    public final static double JoyMaxRange_ = 1.0;        // maximum input range of joysticks
    public final static double JoyKneeOneY_ = 0;		  // starts the first leg of the mapping
    public final static double JoyKneeTwoY_ = 0.35;		  
    
    Preferences prefs = Preferences.getInstance();

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
    
    public static boolean prevRefreshPressed = false;
    public static boolean lastTrigger        = false;
    public static boolean prevRaisePressed   = false;
    public static boolean prevLowerPressed   = false;
    public static boolean elevationTarget_	 = true;
    
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
    	tower1.stop();
    	tower2.stop();
    	
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
     * Prints PID values on the smardashboard.
     * 
     */
    public void updatePrefs()
    {
    	//boolean s = true;
        MaxRPM = prefs.getDouble("MaxRPM", 862.0);
        P = prefs.getDouble("P", 0.0);   // can change values from here, press button to activate changes        
        I = prefs.getDouble("I", 0.0);
        D = prefs.getDouble("D", 0.0);
        F = prefs.getDouble("F", 1.0);
        iZone = prefs.getInt("iZone", 0);
        Ramp = prefs.getDouble("Ramp", 1200.0);
        
        prefs.putDouble("P", P);
        prefs.putDouble("I", I);
        prefs.putDouble("D", D);
        prefs.putDouble("F", F);
        prefs.putInt("iZone", iZone);
        prefs.putDouble("Ramp", Ramp);
        
        SmartDashboard.putNumber("CANTalon P", P);  //displays PID values on SmartDash
        SmartDashboard.putNumber("CANTalon I", I);
        SmartDashboard.putNumber("CANTalon D", D);
        SmartDashboard.putNumber("CANTalon F", F);
        SmartDashboard.putNumber("CANTalon iZone", iZone);
        SmartDashboard.putNumber("CANTalon Ramp", Ramp);
        SmartDashboard.putNumber("MaxRPM", MaxRPM);
        
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
        } 
        catch (CANInvalidBufferException ex)
        {
            CANTimeout();
        }
        System.out.println("finished prefs");
    }
    
    /**
     * TODO: print out the encoder values - should be in the CANTalon class
     * alternateInit - setspeed/velocityreference, setencodertype
     * need to tell it we are using quadencoder, using speed or position
     * TODO: useless now, delete
     */

    public void robotInitDummy()
    {
        SmartDashboard.putNumber("CAN timeouts", CANTimeouts);
        boolean CANInit = false;
        CANTimeouts = 0;
        while (CANInit == false)
        {
            try
            {
				// m_telePeriodicLoops = 0;                 // Reset the number of loops in current second
            	// m_dsPacketsReceivedInCurrentSecond = 0;  // Reset the number of dsPackets in current second
            	
            	motor1 = new CANTalon(1);
            	motor2 = new CANTalon(2);
            	motor3 = new CANTalon(3);
            	
            	motor1.changeControlMode(CANTalon.ControlMode.Speed);
            	motor2.changeControlMode(CANTalon.ControlMode.Speed);
            	motor3.changeControlMode(CANTalon.ControlMode.Speed);
            	
            	motor1.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
            	motor2.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
            	motor3.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
            	
            	motor1.setVoltageRampRate(15); // TODO: put the correct voltage
            	motor2.setVoltageRampRate(15); // TODO: put the correct voltage
            	motor3.setVoltageRampRate(15); // TODO: put the correct voltage
            	
            	//motor1. // TODO: configure degrees per second
            	
            	
                
                //motor1.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);    //chooses which kind of encoder to determine speed feedback
                //motor2.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
                //motor3.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
                //motor1.setVoltageRampRate(15); 
                //motor2.setVoltageRampRate(15);
                //motor3.setVoltageRampRate(15);
                //motor1.configEncoderCodesPerRev(256);   //counts pulses per revolution
                //motor2.configEncoderCodesPerRev(256);
                //motor3.configEncoderCodesPerRev(256);
                //StartPosition = motor1.getPosition();
            	            	
                updatePrefs(); // TODO: set P, I, D, F
                                
                motor1.set(0);
                motor2.set(0);
                motor3.set(0);

                motor1.enableControl(); //starts feedback ctrl
                motor2.enableControl();
                motor3.enableControl();
                
                CANInit = true;
            } 
            catch (CANInvalidBufferException ex)
            {
                CANTimeout();
            }
        }
        System.out.println("### GOT EM' COACH  ###");
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
    	boolean refreshPressed = refreshPrefs.get();

    	if(refreshPressed && (prevRefreshPressed == false))
    	{
    		updatePrefs();
    		tower1.stop();
    		tower2.stop();
    	}
    	prevRefreshPressed = refreshPressed;
    	
    	if(tower1.state_ == State.STOPPED)
    	{
    		boolean raisePressed = raise.get();				// reads button values
	    	if(raisePressed && (prevRaisePressed == false))
	    	{
	    		elevationTarget_ = !elevationTarget_;
	    		tower1.goUp(elevationTarget_);
	    		tower2.goUp(elevationTarget_);
	    	}
	    	prevRaisePressed = raisePressed;
	
	    	boolean lowerPressed = lower.get();
	    	if(lowerPressed && (prevLowerPressed == false))
	    	{
	    		elevationTarget_ = !elevationTarget_;
	    		tower1.goDown(elevationTarget_);
	    		tower2.goDown(elevationTarget_);
	    	}
	    	prevLowerPressed = lowerPressed;
    	}
    	
    	periodic();
    	
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
    	tower1.periodic();
    	tower2.periodic();
    	SmartDashboard.putBoolean("Elevation State1", tower1.getElevaitonState());
    	SmartDashboard.putBoolean("Elevation State2", tower2.getElevaitonState());
    	System.out.println(motor1.getEncVelocity() + "     " + motor2.getEncVelocity() + "     " + motor3.getEncVelocity());
    }
    
    /**
     * Test function for tweaking the towers.
     */
    public void testPeriodic1() 
    {
    	double throttle = driver.getThrottle();
    	
    	if(throttle > 0.5)
    		throttle = 0.5;
    	
    	if(run.get() || (step.get() && hallEffect1.get()))
    	{
    		talon1.set(0.9 * throttle);
    	}
    	else
    	{
    		talon1.set(0.0);
    	}
    	
    	if(run.get() || (step.get() && hallEffect2.get()))
    	{
    		talon2.set(throttle);
    	}
    	else
    	{
    		talon2.set(0.0);
    	}

    	printSensorValues();
    	//Timer.delay(0.5);
    }
    
    static boolean lastTower1 = true;
    static boolean lastTower2 = true;
    /**
     * With the press of the trigger, the tower moves the chain until it hits the edge of the hall effect sensor.
     * outdated now
     */
    public void testPeriodi4()
    
    {
    	boolean readHallEffect1 = hallEffect1.get();
    	boolean readHallEffect2 = hallEffect2.get();
    	boolean triggerPressed = trigger.get();
    	double  throttle = driver.getThrottle();
    	
    	if(throttle > 0.5)
    		throttle = 0.5;
    	
    	if(triggerPressed && (lastTrigger == false))
    	{
    		talon1.set(0.9 * throttle);
    		talon2.set(throttle);
    	}
    	
    	if(readHallEffect1 != lastTower1)
    	{
    		talon1.set(0);
    	}
    	
    	if(readHallEffect2 != lastTower2)
    	{
    		talon2.set(0);
    	}
    	
    	lastTower1 = readHallEffect1;
    	lastTower2 = readHallEffect2;
    	lastTrigger = triggerPressed;
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
     * Function runs the tower talons and prints the boolean of the halleffect to the smartdashboard.
     */
    public void testPeriodic()
    {
    	talon1.set(0.35);
    	talon2.set(0.35);
    	SmartDashboard.putBoolean("HallEffect1", hallEffect1.get());
    	SmartDashboard.putBoolean("HallEffect2", hallEffect2.get());
    }
    
    public void testInit()
    {
    	//motor1.set(MaxRPM);
    }
    
    /**
     * Prints halifax values.
     */
    public void printSensorValues()
    {
    	System.out.println(hallEffect1.get() + "                    " + hallEffect2.get());
    }
}
