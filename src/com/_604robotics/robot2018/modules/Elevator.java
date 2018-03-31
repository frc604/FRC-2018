package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.controller.ClampedIntegralPIDController;
import com._604robotics.robotnik.prefabs.devices.TalonPWMEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Elevator extends Module {
    private final WPI_TalonSRX motorA = new WPI_TalonSRX(Ports.ELEVATOR_MOTOR_A);
    private final WPI_TalonSRX motorB = new WPI_TalonSRX(Ports.ELEVATOR_MOTOR_B);

    public final TalonPWMEncoder encoder = new TalonPWMEncoder(motorA);
    public final Output<Double> encoderRate = addOutput("Elevator Rate", encoder::getVelocity);
    public final Output<Double> encoderClicks = addOutput("Elevator Clicks", encoder::getPosition);

    private final ClampedIntegralPIDController pid = new ClampedIntegralPIDController(
            Calibration.ELEVATOR_P,
            Calibration.ELEVATOR_I,
            Calibration.ELEVATOR_D,
            encoder,
            motorA,
            Calibration.ELEVATOR_PID_PERIOD);
    public final Output<Double> pidError = addOutput("Elevator PID Error", pid::getError);

    public class Idle extends Action {
        private Idle () {
            super(Elevator.this, Idle.class);
        }

        @Override
        protected void run () {
            motorA.set(0);
        }
    }
    public final Idle idle = new Idle();

    public class Manual extends Action {
        public final Input<Double> power;

        public Manual () {
            this(0);
        }

        public Manual (double defaultPower) {
            super(Elevator.this, Manual.class);
            power = addInput("power", defaultPower);
        }

        @Override
        public void run () {
            motorA.set(power.get());
            if (encoder.getPosition() < -Calibration.ELEVATOR_RESET_TOLERANCE) {
                encoder.zero();
            }
        }
    }

    public class Setpoint extends Action {
        private final double targetClicks;

        private Setpoint (double targetClicks) {
            super(Elevator.this, Setpoint.class);
            this.targetClicks = targetClicks;
        }

        @Override
        public void begin () {
            pid.setSetpoint(targetClicks);
            pid.enable();
        }

        @Override
        public void end () {
            pid.reset();
        }
    }
    public final Setpoint low = new Setpoint(Calibration.ELEVATOR_LOW_TARGET);
    public final Setpoint raise = new Setpoint(Calibration.ELEVATOR_RAISE_TARGET);
    public final Setpoint mid = new Setpoint(Calibration.ELEVATOR_MID_TARGET);
    public final Setpoint high = new Setpoint(Calibration.ELEVATOR_HIGH_TARGET);

    public class Hold extends Action {
        private double targetClicks;

        private Hold () {
            super(Elevator.this, Hold.class);
        }

        @Override
        public void begin () {
            targetClicks = encoderClicks.get();
            pid.setErrorSum(Calibration.ELEVATOR_RESET_SUM);
            pid.setSetpoint(targetClicks);
            pid.enable();
        }

        @Override
        public void end () {
            pid.reset();
        }
    }
    public final Hold hold = new Hold();

    public Elevator() {
        super(Elevator.class);

        motorA.setInverted(true);
        motorB.setInverted(true);
        motorB.set(ControlMode.Follower,Ports.ELEVATOR_MOTOR_A);

        encoder.setInverted(false);
        //encoder.setOffset(Calibration.ELEVATOR_ENCODER_ZERO);
        encoder.zero();

        pid.setAbsoluteTolerance(Calibration.ELEVATOR_CLICK_TOLERANCE);
        pid.setIntegralLimits(Calibration.ELEVATOR_MIN_SUM, Calibration.ELEVATOR_MAX_SUM);
        pid.setOutputRange(Calibration.ELEVATOR_MIN_SPEED, Calibration.ELEVATOR_MAX_SPEED);

        setDefaultAction(idle);
    }
}
