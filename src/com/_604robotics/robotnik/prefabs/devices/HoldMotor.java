package com._604robotics.robotnik.prefabs.devices;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;

public class HoldMotor implements PIDOutput {
	
	public SpeedController motor;
	public Encoder encoder;
	public double offset;
	public double increment;
	
	public double upwardsRange;
	public double downwardsRange;
	
	public boolean failsafed;
	
	public double setpoint_offset;
	public double setpoint_increment;
	public double click_tolerance;
	
	public double moving_offset;
	
	public double target_speed;
	
	public boolean at_speed;
	private boolean at_position;
	
	public double elevatorRateOffset() {
		return offset;
	}	
	
	public HoldMotor(SpeedController motor, Encoder encoder) {
		this.motor = motor;
		this.encoder = encoder;
		offset = 0;
		increment = 0.0001;
		
		upwardsRange = 1;
		downwardsRange = 1;
		
		failsafed = false;
		
		setpoint_offset = 0;
		setpoint_increment = 0.0001;
		click_tolerance = 0;
		
		moving_offset = 0;
		
		target_speed = 0.5;
		
		at_speed = false;
		at_position = true;
	}
	
	public HoldMotor(SpeedController motor, Encoder encoder, double target_speed, int click_tolerance) {
		this.motor = motor;
		this.encoder = encoder;
		offset = 0;
		increment = 0.0001;
		
		upwardsRange = 1;
		downwardsRange = 1;
		
		failsafed = false;
		
		setpoint_offset = 0;
		setpoint_increment = 0.0001;
		this.target_speed = target_speed;
		this.click_tolerance = click_tolerance;
		
		at_speed = false;
		at_position = false;
	}
	
	public void set(double output) {
		if( output > 0 ) {
			motor.set(offset + output * upwardsRange);
		} else if( output < 0 ) {
			motor.set(offset + output * downwardsRange);
		} else {
			motor.set(offset);
		}
	}
	
	public void hold() {
		if( !failsafed ) {
			if( encoder.getRate() > 0 ) {
				offset -= increment;
				at_speed = false;
			} else if( encoder.getRate() < 0 ) {
				offset += increment;
				at_speed = false;
			} else {
				at_speed = true;
			}
			if( Math.abs(offset) > 1 ) {
				// means elevator or encoder is broken
				failsafed = true;
				offset = 0;
			}
		}
		upwardsRange = 1-offset;
		downwardsRange = 1+offset;
		set(0);
	}

	public void setpointMove(int click_target) {
		double speed = 0;
		
		if( encoder.get() < click_target - click_tolerance ) {
			speed = target_speed;
		} else if( encoder.get() > click_target + click_tolerance ) {
			speed = -target_speed;
		}
		
		if( encoder.getRate() > speed ) {
			moving_offset -= increment;
			at_speed = false;
		} else if( encoder.getRate() < speed ) {
			moving_offset += increment;
			at_speed = false;
		} else {
			at_speed = true;
		}
		
		if( moving_offset > 1 ) {
			moving_offset = 1;
		} else if( moving_offset < -1 ) {
			moving_offset = -1;
		}
		
		motor.set(moving_offset + speed);
	}
	
	public boolean canHold(int click_target) {
		if( click_target + click_tolerance > encoder.get() && encoder.get() > click_target - click_tolerance ) {
			at_position = true;
		} else {
			at_position = false;
			}
		return at_position;
	}
	
	@Override
	public void pidWrite(double output) {
		/*if( !failsafed ) {
			if( output != 0 ) {
				set(output);
			} else {
				hold();
			}
		} else {
			set(output);
		}*/
		set(output);
	}
}
