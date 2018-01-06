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
public class AntiWindupPIDController extends PIDController {
    private double m_A = 1;
    private double m_C = 1000;

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
        m_A = kA;
        m_C = kC;
    }
    
    public synchronized double getA() {
        return m_A;
    }
    
    public synchronized double getC() {
        return m_C;
    }

    @Override
    protected void calculate() {
        // TODO Auto-generated method stub
        if (m_pidInput.getPIDSourceType().equals(PIDSourceType.kRate)) {
            super.calculate();
        } else {
            /*if (m_I != 0) {
                double newError = m_totalError + m_error;
                newError = newError * m_A;
                newError = Math.min(m_C, Math.max(-m_C, newError));

                double potentialIGain = newError * m_I;
                if (potentialIGain < m_maximumOutput) {
                    if (potentialIGain > m_minimumOutput) {
                        m_totalError = newError;
                    } else {
                        m_totalError = m_minimumOutput / m_I;
                    }
                } else {
                    m_totalError = m_maximumOutput / m_I;
                }
            }*/
            
            boolean enabled = isEnabled();
            PIDSource pidInput;

            synchronized (this) {
                if (m_pidInput == null) {
                    return;
                }
                if (m_pidOutput == null) {
                    return;
                }
                pidInput = m_pidInput; // take snapshot of these values...
            }
            if (enabled) {
                double input;
                double result;
                final PIDOutput pidOutput;
                synchronized (this) {
                    input = pidInput.pidGet();
                }
      /*
      synchronized (this) {
        m_error = getContinuousError(m_setpoint - input);

        if (m_pidInput.getPIDSourceType().equals(PIDSourceType.kRate)) {
          if (m_P != 0) {
            double potentialPGain = (m_totalError + m_error) * m_P;
            if (potentialPGain < m_maximumOutput) {
              if (potentialPGain > m_minimumOutput) {
                m_totalError += m_error;
              } else {
                m_totalError = m_minimumOutput / m_P;
              }
            } else {
              m_totalError = m_maximumOutput / m_P;
            }

            m_result = m_P * m_totalError + m_D * m_error
                + calculateFeedForward();
          }
        } else {
          if (m_I != 0) {
            double potentialIGain = (m_totalError + m_error) * m_I;
            if (potentialIGain < m_maximumOutput) {
              if (potentialIGain > m_minimumOutput) {
                m_totalError += m_error;
              } else {
                m_totalError = m_minimumOutput / m_I;
              }
            } else {
              m_totalError = m_maximumOutput / m_I;
            }
          }

          m_result = m_P * m_error + m_I * m_totalError
              + m_D * (m_error - m_prevError) + calculateFeedForward();
        }
        m_prevError = m_error;

        if (m_result > m_maximumOutput) {
          m_result = m_maximumOutput;
        } else if (m_result < m_minimumOutput) {
          m_result = m_minimumOutput;
        }
        pidOutput = m_pidOutput;
        result = m_result;

        // Update the buffer.
        m_buf.add(m_error);
        m_bufTotal += m_error;
        // Remove old elements when the buffer is full.
        if (m_buf.size() > m_bufLength) {
          m_bufTotal -= m_buf.remove();
        }
      }
*/
                //m_pidOutput.pidWrite(result);
            }
        }
    }
}
