package com._604robotics.robotnik.prefabs.devices;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class TalonPWMEncoder implements PIDSource {
	private final CANTalon talon;
	private PIDSourceType sourceType;

	public TalonPWMEncoder (CANTalon talon) {
		this(talon, PIDSourceType.kDisplacement);
	}

	public TalonPWMEncoder (CANTalon talon, PIDSourceType sourceType) {
		this.talon = talon;
		this.sourceType = sourceType;
	}
	
	public double getPosition () {
		return talon.getPulseWidthPosition();
	}
	
	public double getVelocity () {
		return talon.getPulseWidthVelocity();
	}

	@Override
	public double pidGet () {
		if (sourceType.equals(PIDSourceType.kDisplacement)) {
			return getPosition();
		} else if (sourceType.equals(PIDSourceType.kRate)) {
			return getVelocity();
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	@Override
	public PIDSourceType getPIDSourceType () {
		return sourceType;
	}

	@Override
	public void setPIDSourceType (PIDSourceType sourceType) {
		this.sourceType = sourceType;
	}
}


