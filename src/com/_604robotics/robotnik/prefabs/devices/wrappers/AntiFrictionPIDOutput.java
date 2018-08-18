package com._604robotics.robotnik.prefabs.devices.wrappers;

import com._604robotics.robotnik.utils.annotations.Untested;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * A motor class that feeds a multiple of the power to overcome static friction
 */
@Deprecated @Untested("All is intuitive and theoretical")
public class AntiFrictionPIDOutput implements SpeedController, PIDOutput{
    private final SpeedController origController;
    private final Encoder enc;
    private final double multiplier; 

    /**
     * Constructor to specify the controller with a default multiplier of 1.5 and no encoder
     * @param orig the motor controller
     */
    public AntiFrictionPIDOutput(SpeedController orig) {
        this(orig,null,1.5);
    }
    
    /**
     * Constructor to specify the controller with encoder and a default multiplier of 1.5
     * @param orig the motor controller
     * @param encoder the encoder
     */
    public AntiFrictionPIDOutput(SpeedController orig, Encoder encoder) {
        this(orig,encoder,1.5);
    }

    /**
     * Constructor to specify the controller with a multiplier without an encoder
     * @param orig the motor controller
     */
    public AntiFrictionPIDOutput(SpeedController orig, double multiplier) {
        this(orig,null,multiplier);
    }
    
    /**
     * Constructor to specify both the controller and the multiplier
     * @param orig the motor controller
     * @param the encoder
     * @param multiplier the multiplier to overcome static friction
     */
    public AntiFrictionPIDOutput(SpeedController orig, Encoder encoder, double multiplier) {
        origController = orig;
        enc = encoder;
        this.multiplier = Math.abs(multiplier);
    }
    
    @Override
    public void pidWrite(double output) {
        boolean still;
        if (enc==null) {
            still=origController.get()==0;
        } else {
            still=enc.getStopped();
        }
        if (still) {
            output*=multiplier;
        }
        // Initial jerk is supposed to "unstick" things and thus does not respect original bounds
        pidWrite(clamp(output,-1,1));
    }

    @Override
    public void set(double speed) {
        pidWrite(speed);
    }

    @Override
    public double get() {
        return origController.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        origController.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return origController.getInverted();
    }

    @Override
    public void disable() {
        origController.disable();
    }

    @Override
    public void stopMotor() {
        origController.stopMotor();
    }
    
    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }
}
