package com._604robotics.robotnik.prefabs.devices.wrappers;

import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import com._604robotics.robotnik.utils.annotations.Untested;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * A motor class that ramps changes in input and output.
 */
@Deprecated @Untested("Test wrapper with lone motor controller")
public class RampMotor implements SpeedController, PIDOutput {
    private SpeedController control;
    private SmartTimer timerObj;
    private final double maxRate;

    /**
     * Constructor that uses a default change rate of 4 units per second.
     * @param controller the motor controller
     */
    public RampMotor(SpeedController controller) {
        this(controller, 4);
    }

    /**
     * Constructor to specify the controller and the change rate
     * @param controller the motor controller
     * @param maxChange the change rate at maximum change per second
     */
    public RampMotor(SpeedController controller, double maxChange) {
        control = controller;
        maxRate = Math.abs(maxChange);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void pidWrite(double output) {
        timerObj.startIfNotRunning();
        double maxChange = timerObj.get()*maxRate;
        double out = Math.max(output, get()-maxChange);
        out = Math.min(output, get()+maxChange);
        control.set(out);
        if (timerObj.isRunning()) {
            timerObj.stopAndReset();
        }
        timerObj.start();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void set(double speed) {
        pidWrite(speed);
    }

    @Override
    public double get() {
        return control.get();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setInverted(boolean isInverted) {
        control.setInverted(isInverted);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean getInverted() {
        return control.getInverted();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void disable() {
        control.disable();
        timerObj.stopAndReset();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void stopMotor() {
        control.stopMotor();
    }
}
