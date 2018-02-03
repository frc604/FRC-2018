package com._604robotics.robotnik.prefabs.devices;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PWMSpeedController;

public class RateLimitedMotor implements PIDOutput {
	private double nominalRate;
	private double tolerance;
	private double maxPowerUp;
	private double maxPowerDown;
	private double rampRate;
	private Encoder encoder;
	private PWMSpeedController motor;
	
	public RateLimitedMotor(Encoder encoder, double nominalRate, double tolerance, PWMSpeedController motor){
		this.encoder = encoder;
		this.motor = motor;
		this.nominalRate = nominalRate;
		this.tolerance = tolerance;
		this.maxPowerUp = 1;
		this.maxPowerDown = -1;
		this.rampRate = 0.05;
	}
	public void set(double power){
		/* FORWARD */
		// if faster than target and moving, reduce upwards maxPower
		if(encoder.getRate() > nominalRate && encoder.getRate() > 0) {
			maxPowerUp -= rampRate;
		}
		// if slower than target minus tolerance and moving, increase upwards maxPower
		if(encoder.getRate() < nominalRate - tolerance && encoder.getRate() > 0) {
			maxPowerUp += rampRate;
		}
		
		/* BACKWARD */
		// if faster than target and moving, reduce maxPowerDown
		if(encoder.getRate() < -nominalRate && encoder.getRate() < 0) {
			maxPowerDown += rampRate;
		}
		// if slower than target minus tolerance and moving, increase maxPowerDown
		if(encoder.getRate() > -nominalRate + tolerance && encoder.getRate() < 0) {
			maxPowerDown -= rampRate;
		}
		
		/* Max Power Control */
		// if moving upwards, increase downwards max power
		if(encoder.getRate() >= 0) {
			maxPowerDown -= rampRate;
		}
		// if moving downwards, decrease upwards max power
		if(encoder.getRate() <= 0) {
			maxPowerUp += rampRate;
		}
		/*
		 * WHO CODED THIS AND WHY AREN'T THESE USING JUST > and < FFS
		 */
		
		/* Power Cap */
		// Cap Max Power at 1
		if(maxPowerUp > 1) {
			maxPowerUp = 1;
		}
		if(maxPowerDown < -1) {
			maxPowerDown = -1;
		}
		
		/* Set Power */
		// please dont use this weird notation what are we first robotics
		motor.set((power > 0) ? 									// if power > 0
				((power > maxPowerUp) ? maxPowerUp : power) :		// then use slower of power and maxpowerup
				((power < maxPowerDown) ? maxPowerDown : power));	// else use slower of power and maxpowerdown
	}
	public void stopped(){
		maxPowerUp = 0.1;
		maxPowerDown = 0;
		motor.stopMotor();
	}
	public void setRate(double nominalRate){
		this.nominalRate = nominalRate;
	}
	public void setRampRate(double rampRate){
		this.rampRate = rampRate;
	}
	public void pidWrite(double output) {
		set(output);
	}
}