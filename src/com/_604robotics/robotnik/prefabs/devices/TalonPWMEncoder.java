package com._604robotics.robotnik.prefabs.devices;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class TalonPWMEncoder implements PIDSource {
    // Use TalonSRX because WPI_TalonSRX extends this
    private final TalonSRX talon;
    private PIDSourceType sourceType;
    public enum EncoderType {
        ABSOLUTE,
        RELATIVE
    }
    private final EncoderType encoderType;

    public TalonPWMEncoder (TalonSRX talon) {
        this(talon, PIDSourceType.kDisplacement);
    }
    
    public TalonPWMEncoder (TalonSRX talon, PIDSourceType sourceType) {
        this(talon,sourceType,EncoderType.ABSOLUTE);
    }

    public TalonPWMEncoder (TalonSRX talon, PIDSourceType sourceType, EncoderType type) {
        this.talon = talon;
        this.sourceType = sourceType;
        this.encoderType = type;
    }

    // Use quadrature output for relative and pulse width output for absolute
    public double getPosition () {
        if (encoderType==EncoderType.ABSOLUTE) {
            return talon.getSensorCollection().getPulseWidthPosition();
        } else {
            return talon.getSensorCollection().getQuadraturePosition();
        }
    }

    // Use quadrature output for relative and pulse width output for absolute
    public double getVelocity () {
        if (encoderType==EncoderType.ABSOLUTE) {
            return talon.getSensorCollection().getPulseWidthVelocity();
        } else {
            return talon.getSensorCollection().getQuadratureVelocity();
        }
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


