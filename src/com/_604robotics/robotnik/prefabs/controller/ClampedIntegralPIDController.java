package com._604robotics.robotnik.prefabs.controller;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

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
    protected synchronized double modifyTotalError(double totalError) {
        return clamp(totalError, minIntegral, maxIntegral);
    }
    @Override
    protected synchronized double calculateProportional(double p, double error) {
        double val=super.calculateProportional(p, error);
        System.out.println("p is "+p+", error is "+error+", term is "+val);
        return val;
    };
    
    @Override
    protected double calculateIntegral(double i, double totalError) {
        double val= super.calculateIntegral(i, totalError);
        System.out.println("i is "+i+", totalError is "+totalError+", term is "+val);
        return val;
    }
    
    @Override
    protected synchronized double calculateDerivative(double d, double derror) {
        double val= super.calculateDerivative(d, derror);
        System.out.println("d is "+d+", delta is "+derror+", term is "+val);
        return val;
    }

}
