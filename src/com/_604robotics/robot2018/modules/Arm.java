package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.controller.RotatingArmPIDController;
import com._604robotics.robotnik.prefabs.devices.TalonPWMEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

public class Arm extends Module {
    private final WPI_TalonSRX motorA = new WPI_TalonSRX(Ports.ARM_MOTOR_A);
    private final WPI_TalonSRX motorB = new WPI_TalonSRX(Ports.ARM_MOTOR_B);

    private final TalonPWMEncoder encoder = new TalonPWMEncoder(motorB);
    public final Output<Double> encoderRate = addOutput("encoderRate", 0d, encoder::getVelocity);
    public final Output<Double> encoderClicks = addOutput("encoderClicks", 0d, encoder::getPosition);
    public final Output<Boolean> raised = addOutput("raised", false,
            () -> encoderClicks.get() >= Calibration.ARM_RAISE_TARGET);

    private final DigitalInput bottomLimit = new DigitalInput(Ports.ARM_BOTTOM_SWITCH);
    public final Output<Boolean> atBottomLimit = addOutput("atBottomLimit", false, bottomLimit::get);

    private final RotatingArmPIDController pid = new RotatingArmPIDController(
            Calibration.ARM_P,
            Calibration.ARM_I,
            Calibration.ARM_D,
            Calibration.ARM_F,
            encoder,
            motorA,
            Calibration.ARM_PID_PERIOD);

    public class Idle extends Action {
        private Idle () {
            super(Arm.this, Idle.class);
        }

        @Override
        public void run () {
            motorA.set(0);
        }
    }
    public final Idle idle = new Idle();

    public class Zero extends Action {
        private Zero () {
            super(Arm.this, Zero.class);
        }

        @Override
        public void run () {
            motorA.set(0);
            zeroEncoder();
        }
    }
    public final Zero zero = new Zero();

    public class Manual extends Action {
        public final Input<Double> power;

        public Manual () {
            this(0);
        }

        public Manual (double defaultPower) {
            super(Arm.this, Manual.class);
            power = addInput("power", defaultPower);
        }

        @Override
        public void run () {
            motorA.set(power.get());
        }
    }

    public class Setpoint extends Action {
        public final double targetClicks;

        private Setpoint (double targetClicks) {
            super(Arm.this, Setpoint.class);
            this.targetClicks = targetClicks;
        }
        
        @Override
        public void begin() {
            pid.setSetpoint(targetClicks);
            pid.enable();
        }

        @Override
        public void end () {
            pid.reset();
        }
    }
    public final Setpoint low = new Setpoint(Calibration.ARM_LOW_TARGET);
    public final Setpoint raise = new Setpoint(Calibration.ARM_RAISE_TARGET);
    public final Setpoint mid = new Setpoint(Calibration.ARM_MID_TARGET);
    public final Setpoint high = new Setpoint(Calibration.ARM_HIGH_TARGET);

    public class Hold extends Action {
        private double targetClicks;

        private Hold () {
            super(Arm.this, Hold.class);
        }

        @Override
        public void begin () {
            targetClicks = encoderClicks.get();
            pid.setErrorSum(Calibration.ARM_RESET_SUM);
            pid.setSetpoint(targetClicks);
            pid.enable();
        }

        @Override
        public void end () {
            pid.reset();
        }
    }
    public final Hold hold = new Hold();

    public Arm() {
        super(Arm.class);

        motorA.setInverted(true);
        motorB.setInverted(false);
        motorB.set(ControlMode.Follower,Ports.ARM_MOTOR_A);

        encoder.setInverted(true);
        zeroEncoder();

        pid.setEncoderPeriod(Calibration.ARM_ENCODER_FULL_ROT);
        pid.setIntegralLimits(Calibration.ARM_MIN_SUM, Calibration.ARM_MAX_SUM);
        pid.setOutputRange(Calibration.ARM_MIN_SPEED, Calibration.ARM_MAX_SPEED);
        pid.setAbsoluteTolerance(Calibration.ARM_CLICK_TOLERANCE);

        setDefaultAction(idle);
    }

    private void zeroEncoder () {
        encoder.zero(Calibration.ARM_BOTTOM_LOCATION);
    }
}
