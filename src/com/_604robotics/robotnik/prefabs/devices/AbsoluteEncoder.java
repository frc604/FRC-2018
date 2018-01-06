package com._604robotics.robotnik.prefabs.devices;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public abstract class AbsoluteEncoder implements PIDSource {

    /**
     * Sets the zero of the encoder to its current value.
     */
    public abstract void setZero();

    /**
     * Sets the zero of an encoder.
     * 
     * @param zero
     *            Zero value to set.
     */
    public abstract void setZero(double zero);

    /**
     * Sets the zero of the encoder to an angle.
     * 
     * @param zeroAngle
     *            Zero angle to set.
     */
    public abstract void setZeroAngle(double zeroAngle);

    /**
     * Gets the (raw, non-zeroed) voltage value of the encoder.
     * 
     * @return The (raw, non-zeroed) voltage value of the encoder.
     */
    public abstract double getRawVoltage();

    /**
     * Gets the (zeroed) voltage value of the encoder.
     * 
     * @return The (zeroed) voltage value of the encoder.
     */
    public abstract double getVoltage();

    /**
     * Gets the (zeroed) angle of the encoder.
     * 
     * @return The (zeroed) angle of the encoder.
     */
    public abstract double getAngle();

    /**
     * Gets the (raw, non-zeroed) angle value of the encoder.
     * 
     * @return The (raw, non-zeroed) angle of the encoder.
     */
    public abstract double getRawAngle();

    /**
     * Implementer of the pidGet interface. Make sure to return angles if overridden!
     */
    @Override
    public double pidGet() {
        return getAngle();
    }

    // AbsoluteEncoders only allow kDisplacement
    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public void setPIDSourceType(PIDSourceType sourceType) {
        if (sourceType != PIDSourceType.kDisplacement) {
            throw new IllegalArgumentException("AS5145B class only implements PIDSourceType.kDisplacement");
        }
    }

}