package com._604robotics.robotnik.prefabs.controller;

import com._604robotics.robotnik.utils.annotations.Untested;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

@Deprecated @Untested("The integral term needs to be verified to not blow up")
public class ClampedIntegralPIDController extends ExtendablePIDController {
    
    public double minIntegral = -10;
    public double maxIntegral = 10;

    public ClampedIntegralPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource source, PIDOutput output,
            double period) {
        super(Kp, Ki, Kd, Kf, source, output, period);
        // TODO Auto-generated constructor stub
    }

    public ClampedIntegralPIDController(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output,
            double period) {
        super(Kp, Ki, Kd, source, output, period);
        // TODO Auto-generated constructor stub
    }

    public ClampedIntegralPIDController(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output) {
        super(Kp, Ki, Kd, source, output);
        // TODO Auto-generated constructor stub
    }

    public ClampedIntegralPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource source,
            PIDOutput output) {
        super(Kp, Ki, Kd, Kf, source, output);
        // TODO Auto-generated constructor stub
    }
    
    public void setIntegralLimits(double limitmin, double limitmax) {
        minIntegral=limitmin;
        maxIntegral=limitmax;
    }
    
    @Override
    protected double calculateIntegral(double i, double totalError) {
        return clamp(i*totalError,minIntegral,maxIntegral);
    }

}
