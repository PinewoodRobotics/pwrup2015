package org.usfirst.frc.team4765.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;

/**
 * @author Pavel Khokhlov
 * @author Dean Reece
 * 
 * @version 14 February 2015
 */

public class PIDTower 
{
	public Talon talon_;
	public DigitalInput hallEffect_;
	public DigitalInput heightLimit_;
	public Encoder encoder_;
	public PIDController controller_;
	
	public double setPoint_;
	public double encoderMax_;		// keeps track of encoder value when stop is hit
	public boolean elevationState_ = false; // true = PLATFORM, false = FLOOR  |   OUR TARGET
	
	public static final double elevationDiff = 130.0;
	public static final double StoryDiff = 1024.0;
	
	public PIDTower(int talonPort, int hallEffectPort, int QA, int QB)
	{
		talon_ = new Talon(talonPort);
		hallEffect_ = new DigitalInput(hallEffectPort);
		encoder_ = new Encoder(QA, QB, false);
		encoder_.reset();
		controller_ = new PIDController(0.0, 0.0, 0.0, encoder_, talon_);
	}
	
	public void homeEncoder()
	{
		encoder_.reset();
		setPoint_ = 0.0;
	}
	
	public void setElevationState(boolean elevation)
	{
		if(elevation == elevationState_)
		{
			return;
		}
		
		if(elevation == true)
		{
			controller_.setSetpoint(controller_.getSetpoint() + elevationDiff);
		}
		
		if(elevation == false)
		{
			controller_.setSetpoint(controller_.getSetpoint() - elevationDiff);
		}
	}
	
	public void goUpStory()
	{
		controller_.setSetpoint(controller_.getSetpoint() + StoryDiff);
	}
	
	public void goDownStory()
	{
		controller_.setSetpoint(controller_.getSetpoint() - StoryDiff);
	}
	
	public boolean getElevationState()
	{
		return elevationState_;
	}
	
	public void periodic()
	{
		
	}
	
}
