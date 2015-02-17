package org.usfirst.frc.team4765.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;

/**
 * @author Pavel Khokhlov pkhokhlov@hotmail.com
 * @author Dean Reece	  dean@reeceweb.com
 * 
 * @version 17 February 2015
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
	
	public final double ElevationDiff = 130.0 / 4.0;
	public final double StoryDiff = 1024.0 / 4.0;
	
	public PIDTower(int talonPort, int hallEffectPort, int QA, int QB)
	{
		talon_ = new Talon(talonPort);
		hallEffect_ = new DigitalInput(hallEffectPort);
		encoder_ = new Encoder(QA, QB, false, EncodingType.k4X);
		controller_ = new PIDController(0.0, 0.0, 0.0, encoder_, talon_);
		controller_.setAbsoluteTolerance(20.0);
	}
	
	/*\][][\*
	 * This function moves the towers up or down a distance corresponding to the platform height.
	 * If the robot is already at platform, the robot does not move the chain up.
	 * If the robot is already at floor, the robot does not move the chaind down.
	 */
	public void setElevationState(boolean elevation)
	{
		if(elevation == elevationState_)
		{
			return;
		}
		
		if(elevation == true)
		{
			goToSetPoint(setPoint_ + ElevationDiff);
		}
		
		if(elevation == false)
		{
			goToSetPoint(setPoint_ - ElevationDiff);
		}
		
		elevationState_ = elevation;
	}
	
	/**
	 * This function makes the towers go up one full rotation - corresponds to going up 1 tote height.
	 */
	public void goUpStory()
	{
		if(heightLimit_ == true)
			return;
		
		goToSetPoint(controller_.getSetpoint() + StoryDiff);
	}
	
	/**
	 * This function makes the towers go down one full rotation - corresponds to going down 1 tote height.
	 */
	public void goDownStory()
	{
		goToSetPoint(controller_.getSetpoint() - StoryDiff);
	}
	
	/**
	 * This function returns what elevation we are at - floor(false) or platform(true).
	 */
	public boolean getElevationState()
	{
		return elevationState_;
	}
	
	/**
	 * This funciton sets how high we can go - based on the switch we hit at the top of the robot.
	 */
	public void setHeightLimit(boolean heightLimit)
	{
		if(heightLimit == heightLimit_)
		{
			return;
		}
		
		heightLimit_ = heightLimit;
		
		if(heightLimit_ == true)
		{
			controller_.setInputRange(setPoint_ - 9999, encoder_.pidGet());	/* if we hit height limit, we allow robot to go down at regular power, 
																			 but don't allow it to go up */
			goToSetPoint(setPoint_);
		}
		else
		{
			controller_.setInputRange(0, 0);
			goToSetPoint(setPoint_);
		}
	}
	
	/**
	 * This function sets our local variable to the point we want to go and makes the controller go to our newly set local variable.
	 */
	public void goToSetPoint(double point)
	{
		setPoint_ = point;
		controller_.setSetpoint(setPoint_ + offSet_);
	}
	
	public boolean readyForCommand()
	{
		if(homing)
			return false;
		
		if(heightLimit_ == true)
			return true;
		
		return controller_.onTarget();
	}
	
	/**
	 * This function disables PID control and makes the towers go down. This function is followed up by periodic().
	 */
	public void goHome()
	{
		controller_.disable();
		talon_.set(-0.3);
		homing = true;
	}
	
	/**
	 * This function is called after goHome() in Robot
	 * This function stops the movement of the towers if the halleffect is detected, thus synchronizing them.
	 */
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
                goToSetPoint(setPoint_);
                //controller_.setSetpoint(setPoint_ + offSet_); // go to the offset number
                controller_.enable();
                homing = false;

			}
		}
		prevHallEffect_ = hallEffect;
	}
	
}
