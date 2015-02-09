package org.usfirst.frc.team4765.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;

public class Tower 
{
	public enum ElevationState
	{
		FLOOR, PLATFORM
	}
	
	public enum State
	{
		RUNUP, RUNDOWN, STOPPED 
	}
	
	/**
	 * ALL OF OUR COMMANDS
	 * 
	 * 	goUp - raise totoes to next magnetic edge
	 * 	TODO: goUpLevel - raise totes a full 42 link (full cycle)
	 * 	goDown - lowers totoes to next magnetic edge
	 * 	TODO: goDownLevel - lower totes a full 42 link (full cycle)
	 * 
	 * 	TODO: implement dashboard feedback
	 * 	
	 * 
	 */
	public final double speedUp_ = 1;
	public final double speedDown_ = - 0.35;
	
	public ElevationState elevationState_ = ElevationState.FLOOR;
	public State state_ = State.STOPPED;
	
	public Talon motor_;
	public DigitalInput hallEffect_;
	public DigitalInput heightLimit_;
	
	public boolean lastHallEffect = false;
	
	public Tower(Talon talon, DigitalInput hallEffect, DigitalInput heightLimit)
	{
		motor_ = talon;
		hallEffect_ = hallEffect;
		heightLimit_ = heightLimit;
	}
	
	
	public void goUp()
	{
		state_ = State.RUNUP;
	}
	
	
	public void goDown()
	{
		state_ = State.RUNDOWN;
	}
	
	public void stop()
	{
		state_ = State.STOPPED;
	}
	
	/**
	 * all logic for state machine is here
	 */
	public void periodic()
	{
		boolean hallEffect = hallEffect_.get();
		boolean heightLimit = heightLimit_.get();
		
		switch(state_)
		{
			case RUNUP:
			{
				if(heightLimit == false)
				{
					state_ = State.STOPPED;
				}
				
				if(hallEffect != lastHallEffect)
				{
					state_ = State.STOPPED;
					
					if(hallEffect == false)
					{
						elevationState_ = ElevationState.FLOOR;
					}
					else
					{
						elevationState_ = ElevationState.PLATFORM;
					}
				}
				
				motor_.set(speedUp_);
			}
			break;
				
			case RUNDOWN:
			{
				if(hallEffect != lastHallEffect)
				{
					state_ = State.STOPPED;
					
					if(hallEffect == true)
					{
						elevationState_ = ElevationState.FLOOR;
					}
					else
					{
						elevationState_ = ElevationState.PLATFORM;
					}
				}
				
				motor_.set(speedDown_);
			}
			break;
			
			default:
			case STOPPED:
			{
				motor_.set(0);
			}
			break;
		}
		lastHallEffect = hallEffect;
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
}
