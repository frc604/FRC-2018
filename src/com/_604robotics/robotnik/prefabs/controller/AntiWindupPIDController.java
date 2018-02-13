package com._604robotics.robotnik.prefabs.controller;

import com._604robotics.robotnik.utils.annotations.Untested;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;

//Untested (algorithm used previously on 2014 but port INCOMPLETE) 
//             _            _           _
// _   _ _ __ | |_ ___  ___| |_ ___  __| |
//| | | | '_ \| __/ _ \/ __| __/ _ \/ _` |
//| |_| | | | | ||  __/\__ \ ||  __/ (_| |
// \__,_|_| |_|\__\___||___/\__\___|\__,_|
/**
 * <p>
 * Implements an anti-windup PID controller.
 * </p>
 * 
 * To prevent windup, the integral sum is capped at +-Kc.
 * The integral sum is also multiplied by m_A each iteration, so that the sum
 * will decay exponentially.
 * This prevents the integral term from becoming too large.
 */
@Deprecated @Untested("Incomplete port from 2014 codebase")
public class AntiWindupPIDController extends ClampedIntegralPIDController {
    private double m_A = 1;
    private double m_C = 10;

    // Pass through constructors
    /**
     * {@inheritDoc}
     */
    @Untested("Incomplete port from 2014 codebase")
    public AntiWindupPIDController(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output) {
        super(Kp, Ki, Kd, source, output);
    }

    /**
     * {@inheritDoc}
     */
    @Untested("Incomplete port from 2014 codebase")
    public AntiWindupPIDController(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output, double period) {
        super(Kp, Ki, Kd, source, output, period);
    }

    /**
     * {@inheritDoc}
     */
    @Untested("Incomplete port from 2014 codebase")
    public AntiWindupPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource source, PIDOutput output) {
        super(Kp, Ki, Kd, Kf, source, output);
    }

    /**
     * {@inheritDoc}
     */
    @Untested("Incomplete port from 2014 codebase")
    public AntiWindupPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource source, PIDOutput output,
            double period) {
        super(Kp, Ki, Kd, Kf, source, output, period);
    }

    // Ka and Kc constructors
    @Untested("Incomplete port from 2014 codebase")
    public AntiWindupPIDController(double Kp, double Ki, double Ka, double Kc, double Kd, PIDSource source,
            PIDOutput output) {
        super(Kp, Ki, Kd, source, output);
        if (Ka>1 || Ka<=0) {
            throw new IllegalArgumentException("Ka must be between 0 and 1!");
        }
        if (Kc<=0) {
            throw new IllegalArgumentException("Kc must be greater than 0!");
        }
        m_A = Ka;
        m_C = Kc;
        setIntegralLimits(-m_C,m_C);
    }

    @Untested("Incomplete port from 2014 codebase")
    public AntiWindupPIDController(double Kp, double Ki, double Ka, double Kc, double Kd, PIDSource source,
            PIDOutput output, double period) {
        super(Kp, Ki, Kd, source, output, period);
        if (Ka>1 || Ka<=0) {
            throw new IllegalArgumentException("Ka must be between 0 and 1!");
        }
        if (Kc<=0) {
            throw new IllegalArgumentException("Kc must be greater than 0!");
        }
        m_A = Ka;
        m_C = Kc;
        setIntegralLimits(-m_C,m_C);
    }

    @Untested("Incomplete port from 2014 codebase")
    public AntiWindupPIDController(double Kp, double Ki, double Ka, double Kc, double Kd, double Kf, PIDSource source,
            PIDOutput output) {
        super(Kp, Ki, Kd, Kf, source, output);
        if (Ka>1 || Ka<=0) {
            throw new IllegalArgumentException("Ka must be between 0 and 1!");
        }
        if (Kc<=0) {
            throw new IllegalArgumentException("Kc must be greater than 0!");
        }
        m_A = Ka;
        m_C = Kc;
        setIntegralLimits(-m_C,m_C);
    }

    @Untested("Incomplete port from 2014 codebase")
    public AntiWindupPIDController(double Kp, double Ki, double Ka, double Kc, double Kd, double Kf, PIDSource source,
            PIDOutput output, double period) {
        super(Kp, Ki, Kd, Kf, source, output, period);
        if (Ka>1 || Ka<=0) {
            throw new IllegalArgumentException("Ka must be between 0 and 1!");
        }
        if (Kc<=0) {
            throw new IllegalArgumentException("Kc must be greater than 0!");
        }
        m_A = Ka;
        m_C = Kc;
        setIntegralLimits(-m_C,m_C);
    }

    public synchronized void setPID(double kP, double kI, double kA, double kC, double kD) {
        setPID(kP, kI, kD);
        setAC(kA, kC);
    }

    public synchronized void setPID(double kP, double kI, double kA, double kC, double kD, double Kf) {
        setPID(kP, kI, kD, Kf);
        setAC(kA, kC);
    }
    
    public synchronized void setAC(double kA, double kC) {
        m_thisMutex.lock();
        try {
            m_A = kA;
            m_C = kC;
            setIntegralLimits(-m_C,m_C);
        } finally {
            m_thisMutex.unlock();
        }
    }
    
    public synchronized double getA() {
        m_thisMutex.lock();
        try {
            return m_A;
        } finally {
            m_thisMutex.unlock();
        }
    }
    
    public synchronized double getC() {
        m_thisMutex.lock();
        try {
            return m_C;
        } finally {
            m_thisMutex.unlock();
        }
    }

    @Override
    protected synchronized double modifyTotalError(double totalError) {
        // TODO: Use proper timer mechanisms
        return super.modifyTotalError(totalError)*m_A;
    }
}
