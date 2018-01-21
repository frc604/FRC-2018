package com._604robotics.robotnik.prefabs.devices;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PWMSpeedController;

public class HoldMotor implements PIDOutput {
	
	private PWMSpeedController motor;
	private Encoder encoder;
	private double offset;
	private double increment;
	
	private double upwardsRange;
	private double downwardsRange;
	
	private boolean failsafed;
	
	public HoldMotor(PWMSpeedController motor, Encoder encoder) {
		this.motor = motor;
		this.encoder = encoder;
		this.offset = 0;
		this.increment = 0.01;
		
		upwardsRange = 1;
		downwardsRange = 1;
		
		failsafed = false;
	}
	
	private void set(double output) {
		if( output > 0 ) {
			motor.set(output * upwardsRange + offset);
		} else if( output < 0 ) {
			motor.set(output * downwardsRange - offset);
		} else {
			motor.set(offset);
		}
	}
	
	private void hold() {
		if( encoder.getRate() > 0 ) {
			offset -= increment;
		} else if( encoder.getRate() < 0 ) {
			offset += increment;
		}
		if( Math.abs(offset) > 1 ) {
			// means elevator or encoder is broken
			failsafed = true;
			offset = 0;
		}
		upwardsRange = 1-offset;
		downwardsRange = -1+offset;
		set(0);
	}
	
	@Override
	public void pidWrite(double output) {
		if( !failsafed ) { 
			if( output != 0 ) {
				set(output);
			} else {
				hold();
			}
		} else {
			set(output);
		}
	}
}
