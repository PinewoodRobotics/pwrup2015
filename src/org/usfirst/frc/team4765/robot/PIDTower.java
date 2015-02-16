package org.usfirst.frc.team4765.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
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
	public Encoder encoder_;
	public PIDController controller_;
	
	public double setPoint_;
	public double encoderMax_;		// keeps track of encoder value when stop is hit
	public double offSet_;
	public boolean prevHallEffect_ = false;
	public boolean elevationState_ = false; // true = PLATFORM, false = FLOOR  |   OUR TARGET
	public boolean heightLimit_ = false; // true = we are touching it, false = we are not touching it
	public boolean homing = false;
	
	public final double elevationDiff = 130.0 / 4.0;
	public final double StoryDiff = 1024.0 / 4.0;
	
	public PIDTower(int talonPort, int hallEffectPort, int QA, int QB)
	{
		talon_ = new Talon(talonPort);
		hallEffect_ = new DigitalInput(hallEffectPort);
		encoder_ = new Encoder(QA, QB, false, EncodingType.k4X);
		controller_ = new PIDController(0.0, 0.0, 0.0, encoder_, talon_);
		controller_.setAbsoluteTolerance(6.0);
	}
	
	public void setElevationState(boolean elevation)
	{
		if(elevation == elevationState_)
		{
			return;
		}
		
		if(elevation == true)
		{
			goToSetPoint(controller_.getSetpoint() + elevationDiff);
		}
		
		if(elevation == false)
		{
			goToSetPoint(controller_.getSetpoint() - elevationDiff);
		}
		
		elevationState_ = elevation;
	}
	
	public void goUpStory()
	{
		if(heightLimit_ == true)
			return;
		
		goToSetPoint(controller_.getSetpoint() + StoryDiff);
	}
	
	public void goDownStory()
	{
		goToSetPoint(controller_.getSetpoint() - StoryDiff);
	}
	
	public boolean getElevationState()
	{
		return elevationState_;
	}
	
	public void setHeightLimit(boolean heightLimit)
	{
		if(heightLimit == heightLimit_)
		{
			return;
		}
		
		heightLimit_ = heightLimit;
		
		if(heightLimit_ == true)
		{
			controller_.setOutputRange(Robot.towerMin, 0);	// if we hit height limit, we allow robot to go down at regular power, 
															// but don't allow it to go up
		}
		else
		{
			controller_.setOutputRange(Robot.towerMin, Robot.towerMax);
			goToSetPoint(setPoint_);
		}
	}
	
	public void goToSetPoint(double point)
	{
		setPoint_ = point;
		controller_.setSetpoint(setPoint_);
	}
	
	public boolean readyForCommand()
	{
		if(homing)
			return false;
		
		if(heightLimit_ == true)
			return true;
		
		return controller_.onTarget();
	}
	
	public void goHome()
	{
		controller_.disable();
		talon_.set(- 0.3);
		homing = true;
	}
	
	public void periodic()
	{
		boolean hallEffect = hallEffect_.get();
		if(homing)
		{
			if((hallEffect == true) && (prevHallEffect_ == false))
			{
                talon_.set(0.0);
                encoder_.reset();
                setPoint_ = 0.0;
                controller_.setSetpoint(setPoint_);
                //controller_.setSetpoint(setPoint_ + offSet_); // go to the offset number
                controller_.enable();
                homing = false;

			}
		}
		prevHallEffect_ = hallEffect;
	}
	
}
