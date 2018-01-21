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
		if(encoder.getRate() > nominalRate && encoder.getRate() > 0) {
			maxPowerUp -= rampRate;
		}
		if(encoder.getRate() < nominalRate - tolerance && encoder.getRate() > 0) {
			maxPowerUp += rampRate;
		}
		if(encoder.getRate() < -nominalRate && encoder.getRate() < 0) {
			maxPowerDown += rampRate;
		}
		if(encoder.getRate() > -nominalRate + tolerance && encoder.getRate() < 0) {
			maxPowerDown -= rampRate;
		}
		if(encoder.getRate() >= 0) {
			maxPowerDown -= rampRate;
		}
		if(encoder.getRate() <= 0) {
			maxPowerUp += rampRate;
		}
		if(maxPowerUp > 1) {
			maxPowerUp = 1;
		}
		if(maxPowerDown < -1) {
			maxPowerDown = -1;
		}
		motor.set((power > 0) ? 
				((power > maxPowerUp) ? maxPowerUp : power) :
				((power < maxPowerDown) ? maxPowerDown : power));
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