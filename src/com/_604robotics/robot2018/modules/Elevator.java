package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Calibration.ElevatorFactors;
import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.controller.ClampedIntegralPIDController;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Elevator extends Module {

    public WPI_TalonSRX motor = new WPI_TalonSRX(Ports.ELEVATOR_MOTOR_A);

    public final Setpoint setpoint = new Setpoint();

    public final Output<Integer> encoderRate = addOutput("Elevator Rate", this::getEncoderRate);
    public final Output<Integer> encoderClicks = addOutput("Elevator Clicks", this::getEncoderPos);

    public boolean holding = true;
    public double power = 0;

    public final Output<Boolean> getHolding = addOutput("Holding", this::getHolding);
    public final Output<Double> getPower = addOutput("Power", this::getPower);

    private final ClampedIntegralPIDController pid;
    private final SmartTimer PIDTimer = new SmartTimer();

    
    public int getEncoderPos() {
        return -motor.getSensorCollection().getPulseWidthPosition(); 
    }
    public int getEncoderRate() {
        return -motor.getSensorCollection().getPulseWidthVelocity(); 
    }

    public boolean getHolding() {
        return holding;
    }

    public double getPower() {
        return power;
    }

    public class Move extends Action {
        public final Input<Double> liftPower;

        public Move() {
            this(0);
        }

        public Move (double power) {
            super(Elevator.this, Move.class);
            liftPower = addInput("Lift Power", power, true);
        }

        @Override
        public void run () {
            holding = false;
            power = liftPower.get();
            motor.set(liftPower.get());
        }
    }

    public class Setpoint extends Action {
        public final Input<Integer> target_clicks;

        public Setpoint() {
            this(0);
        }

        public Setpoint(int clicks) {
            super(Elevator.this, Setpoint.class);
            target_clicks = addInput("Target Clicks", clicks, true);
        }

        @Override
        public void begin() {
            pid.enable();
            holding = false;
        }
        @Override
        public void run () {
            pid.setSetpoint(target_clicks.get());
        }
        @Override
        public void end () {
            PIDTimer.reset();
            pid.reset();
        }
    }

    public Elevator() {
        super(Elevator.class);
        pid = new ClampedIntegralPIDController(ElevatorFactors.ELEVATOR_P,
                ElevatorFactors.ELEVATOR_I,
                ElevatorFactors.ELEVATOR_D,
                new PIDSource() {

                    @Override
                    public void setPIDSourceType(PIDSourceType pidSource) {
                        // Ignore and do nothing
                    }

                    @Override
                    public PIDSourceType getPIDSourceType() {
                        return PIDSourceType.kDisplacement;
                    }

                    @Override
                    public double pidGet() {
                        return getEncoderPos();
                    }
                },
                motor,
                ElevatorFactors.ELEVATOR_PID_PERIOD);
        pid.setIntegralLimits(ElevatorFactors.ELEVATOR_MIN_SUM, ElevatorFactors.ELEVATOR_MAX_SUM);
        pid.setOutputRange(ElevatorFactors.ELEVATOR_MIN_SPEED, ElevatorFactors.ELEVATOR_MAX_SPEED);
        setpoint.target_clicks.set(getEncoderPos());
        setDefaultAction(setpoint);
    }
}
