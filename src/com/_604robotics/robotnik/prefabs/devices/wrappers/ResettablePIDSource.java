package com._604robotics.robotnik.prefabs.devices.wrappers;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * A resettable PID source.
 */
public class ResettablePIDSource implements PIDSource {
    private final PIDSource source;
    private double zero = 0;

    /**
     * Creates a resettable PID source.
     * @param source Base PID source.
     */
    public ResettablePIDSource (PIDSource source) {
        this.source = source;
    }

    /**
     * Creates a resettable PID source.
     * @param source Base PID source.
     * @param zero Reference value to consider as zero.
     */
    public ResettablePIDSource (PIDSource source, double zero) {
        this.source = source;
        this.zero = zero;
    }

    /**
     * Gets the zero value of the PID.
     * @return The PID's zero value.
     */
    public double getZero () {
        return zero;
    }

    /**
     * Sets the zero value of the PID.
     * @param zero Zero value to set.
     */
    public void setZero (double zero) {
        this.zero = zero;
    }

    /**
     * Sets the zero value of the PID to the current raw value.
     */
    public void setZero () {
        setZero(getRaw());
    }

    /**
     * Sets the zero value of the PID to the current raw value offset by the given amount.
     * @param offset Amount to offset the current value by.
     */
    public void setZeroOffset (double offset) {
        setZero(getRaw() + offset);
    }

    /**
     * Sets the zero value of the PID such that it would read the given real value at the current raw value.
     * @param newPosition Real value for the PID to read at the current raw value.
     */
    public void setZeroRel (double newPosition) {
        setZeroRel(getRaw(), newPosition);
    }

    /**
     * Sets the zero value of the PID such that it would read the given real value if it were at the given raw value.
     * @param rawPosition Raw value at which the PID should read the given real value.
     * @param newPosition Real value for the PID to read at the given raw value.
     */
    public void setZeroRel (double rawPosition, double newPosition) {
        setZero(rawPosition - newPosition);
    }

    /**
     * Gets the raw value of the PID.
     * @return The raw value of the PID.
     */
    public double getRaw () {
        return source.pidGet();
    }

    /**
     * Gets the real value of the PID.
     * @return The real value of the PID.
     */
    public double get () {
        return getRaw() - zero;
    }

    @Override
    public double pidGet () {
        return get();
    }

    @Override
    public PIDSourceType getPIDSourceType () {
        return source.getPIDSourceType();
    }

    @Override
    public void setPIDSourceType (PIDSourceType sourceType) {
        source.setPIDSourceType(sourceType);
    }
}