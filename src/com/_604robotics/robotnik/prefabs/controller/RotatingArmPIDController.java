package com._604robotics.robotnik.prefabs.controller;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;


/**
 * <p>Subclass of PIDController that has a feedforward for rotating arms.</p>
 * This subclass requires an PIDSource as the PIDSource and uses only continuous error.
 * Zero is assumed to be horizontal. Users are responsible for properly zeroing the PIDSource beforehand.
 */
public class RotatingArmPIDController extends ClampedIntegralPIDController {

    public RotatingArmPIDController(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output) {
        super(Kp, Ki, Kd, source, output);
        setInputRange(0, 360);
        setContinuous();
    }

    public RotatingArmPIDController(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output,
            double period) {
        super(Kp, Ki, Kd, source, output, period);
        setInputRange(0, 360);
        setContinuous();
    }

    public RotatingArmPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource source, PIDOutput output) {
        super(Kp, Ki, Kd, Kf, source, output);
        setInputRange(0, 360);
        setContinuous();
    }

    public RotatingArmPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource source, PIDOutput output,
            double period) {
        super(Kp, Ki, Kd, Kf, source, output, period);
        setInputRange(0, 360);
        setContinuous();
    }

    @Override
    public synchronized void setContinuous(boolean continuous) {
        if (!continuous) {
            throw new IllegalArgumentException("RotatingArmPIDController must have continuous error!");
        }
        super.setContinuous(continuous);
    }

    @Override
    public synchronized void setContinuous() {
        super.setContinuous();
    }

    /**
     * <p>Overriden feed forward part of PIDController.</p>
     * 
     * This is a physically based model which multiplies feed forward coefficient by cosine.
     * The feedforward calculates the expected torque needed to hold an arm steady, scaled to motor power.
     * 
     *  @return the feed forward value
     */
    @Override
    protected double calculateFeedForward() {
        // Calculate cosine for torque factor
        double angle;
        double fValue;
        double minInput;
        double maxInput;
        m_thisMutex.lock();
        try {
            angle = m_pidInput.pidGet();
            fValue = getF();
            minInput = m_minimumInput;
            maxInput = m_maximumInput;
        } finally {
            m_thisMutex.unlock();
        }
        // Cosine is periodic so sawtooth wraparound is not a concern
        angle/=(maxInput-minInput);
        angle*=(2*Math.PI);
        double cosine = Math.cos(angle);
        return fValue * cosine;
    }

}
