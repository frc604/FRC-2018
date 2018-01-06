package com._604robotics.robotnik.prefabs.devices;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class TalonPWMEncoder implements PIDSource {
    private final TalonSRX talon;
    private PIDSourceType sourceType;

    public TalonPWMEncoder (TalonSRX talon) {
        this(talon, PIDSourceType.kDisplacement);
    }

    public TalonPWMEncoder (TalonSRX talon, PIDSourceType sourceType) {
        this.talon = talon;
        this.sourceType = sourceType;
    }

    // Verify getPulseWidth vs getQuadrature
    public double getPosition () {
        return talon.getSensorCollection().getPulseWidthPosition();
    }

    public double getVelocity () {
        return talon.getSensorCollection().getPulseWidthVelocity();
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


