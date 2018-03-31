package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.devices.wrappers.RampMotor;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Module {
    private final RampMotor m_frontLeft = new RampMotor(new PWMVictorSPX(Ports.DRIVE_FRONT_LEFT_MOTOR),Calibration.DRIVE_MOTOR_RAMP);
    private final RampMotor m_rearLeft = new RampMotor(new PWMVictorSPX(Ports.DRIVE_REAR_LEFT_MOTOR),Calibration.DRIVE_MOTOR_RAMP);
    private final SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

    private final RampMotor m_frontRight = new RampMotor(new PWMVictorSPX(Ports.DRIVE_FRONT_RIGHT_MOTOR),Calibration.DRIVE_MOTOR_RAMP);
    private final RampMotor m_rearRight = new RampMotor(new PWMVictorSPX(Ports.DRIVE_REAR_RIGHT_MOTOR),Calibration.DRIVE_MOTOR_RAMP);
    private final SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

    private final DifferentialDrive robotDrive = new DifferentialDrive(m_left, m_right);

    // Reversed from previously due to new mountings
    private final Encoder encoderLeft = new Encoder(Ports.ENCODER_LEFT_A,
            Ports.ENCODER_LEFT_B,
            false,
            CounterBase.EncodingType.k4X);
    private final Encoder encoderRight = new Encoder(Ports.ENCODER_RIGHT_A,
            Ports.ENCODER_RIGHT_B,
            true,
            CounterBase.EncodingType.k4X);
    
    public void resetEncoders () {
        encoderLeft.reset();
        encoderRight.reset();
    }

    public final Output<Integer> leftClicks = addOutput("leftClicks", encoderLeft::get);
    public final Output<Integer> rightClicks = addOutput("rightClicks", encoderRight::get);
    
    public final Output<Double> leftClickRate = addOutput("leftClickRate", encoderLeft::getRate);
    public final Output<Double> rightClickRate = addOutput("rightClickRate", encoderRight::getRate);

    public class Idle extends Action {
        private Idle () {
            super(Drive.this, Idle.class);
        }

        @Override
        public void run () {
            robotDrive.stopMotor();
        }
    }
    public final Idle idle = new Idle();

    public class TankDrive extends Action {
        public final Input<Double> leftPower;
        public final Input<Double> rightPower;
        public final Input<Boolean> squaredInputs;

        public TankDrive () {
            this(0, 0);
        }

        public TankDrive (boolean defaultSquaredInputs) {
            this(0, 0);
        }

        public TankDrive (double defaultLeftPower, double defaultRightPower) {
            this(defaultLeftPower, defaultRightPower, true);
        }

        public TankDrive (double defaultLeftPower, double defaultRightPower, boolean defaultSquaredInputs) {
            super(Drive.this, TankDrive.class);

            leftPower = addInput("leftPower", defaultLeftPower);
            rightPower = addInput("rightPower", defaultRightPower);
            squaredInputs = addInput("squaredInputs", defaultSquaredInputs);
        }

        @Override
        public void run () {
            robotDrive.tankDrive(leftPower.get(), rightPower.get(), squaredInputs.get());
        }
    }

    public class ArcadeDrive extends Action {
        public final Input<Double> movePower;
        public final Input<Double> rotatePower;
        public final Input<Boolean> squaredInputs;

        public ArcadeDrive () {
            this(0, 0);
        }

        public ArcadeDrive (boolean defaultSquaredInputs) {
            this(0, 0, defaultSquaredInputs);
        }

        public ArcadeDrive (double defaultMovePower, double defaultRotPower) {
            this(defaultMovePower, defaultRotPower, true);
        }

        public ArcadeDrive (double defaultMovePower, double defaultRotPower, boolean defaultSquaredInputs) {
            super(Drive.this, ArcadeDrive.class);
            movePower = addInput("movePower", defaultMovePower);
            rotatePower = addInput("rotatePower", defaultRotPower);
            squaredInputs = addInput("squaredInputs", defaultSquaredInputs);
        }

        @Override
        public void run () {
            robotDrive.arcadeDrive(movePower.get(), rotatePower.get(), squaredInputs.get());
        }
    }

    public Drive () {
        super(Drive.class);

        robotDrive.setDeadband(0.04);

        setDefaultAction(idle);
    }
}
