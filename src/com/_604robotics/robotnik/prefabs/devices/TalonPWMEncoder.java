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
    private boolean inverted;
    private double offset = 0;

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
        this.inverted = false;
    }

    public boolean isInverted() {
        return inverted;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    // Use quadrature output for relative and pulse width output for absolute
    public double getPosition () {
        int multfactor = inverted ? -1 : 1;
        if (encoderType==EncoderType.ABSOLUTE) {
            return multfactor*(talon.getSensorCollection().getPulseWidthPosition()-offset);
        } else {
            return multfactor*talon.getSensorCollection().getQuadraturePosition();
        }
    }

    // Use quadrature output for relative and pulse width output for absolute
    public double getVelocity () {
        int multfactor = inverted ? -1 : 1;
        if (encoderType==EncoderType.ABSOLUTE) {
            return multfactor*talon.getSensorCollection().getPulseWidthVelocity();
        } else {
            return multfactor*talon.getSensorCollection().getQuadratureVelocity();
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

    public void zero() {
        if (encoderType==EncoderType.ABSOLUTE) {
            offset=(talon.getSensorCollection().getPulseWidthPosition());
        }
    }
    
    public void zero(double value) {
        if (encoderType==EncoderType.ABSOLUTE) {
            int multfactor = inverted ? -1 : 1;
            offset=talon.getSensorCollection().getPulseWidthPosition()-(multfactor * value);
        }
    }

    // Use zeroing things instead of setting the offset directly
    @Deprecated
    public void setOffset(double offset) {
        if (encoderType==EncoderType.ABSOLUTE) {
            int multfactor = inverted ? -1 : 1;
            this.offset = multfactor * offset;
        }
    }
    
    public double getOffset() {
        if (encoderType==EncoderType.ABSOLUTE) {
            return offset;
        } else {
            return 0;
        }
    }
}


