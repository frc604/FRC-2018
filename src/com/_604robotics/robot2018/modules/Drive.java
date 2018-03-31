package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.devices.wrappers.ArcadeDrivePIDOutput;
import com._604robotics.robotnik.prefabs.devices.wrappers.RampMotor;

import com._604robotics.robotnik.prefabs.devices.wrappers.SimplePIDSource;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import edu.wpi.first.wpilibj.*;
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
    
    private void resetEncoders () {
        encoderLeft.reset();
        encoderRight.reset();
    }

    public final Output<Integer> leftClicks = addOutput("leftClicks", 0, encoderLeft::get);
    public final Output<Integer> rightClicks = addOutput("rightClicks", 0, encoderRight::get);
    
    public final Output<Double> leftClickRate = addOutput("leftClickRate", 0d, encoderLeft::getRate);
    public final Output<Double> rightClickRate = addOutput("rightClickRate", 0d, encoderRight::getRate);

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

    public class ArcadeServo extends Action {
        private final PIDSource encoderDiffPIDSource = new SimplePIDSource(PIDSourceType.kDisplacement) {
            @Override
            public double pidGet () {
                return -rightClicks.get() + leftClicks.get();
            }
        };

        private final PIDSource encoderAvgPIDSource = new SimplePIDSource(PIDSourceType.kDisplacement) {
            @Override
            public double pidGet () {
                return (rightClicks.get() + leftClicks.get()) / 2;
            }
        };

        private final ArcadeDrivePIDOutput arcadeDrivePIDOutput = new ArcadeDrivePIDOutput(robotDrive);

        private final PIDController moveController = new PIDController(
                Calibration.DRIVE_MOVE_PID_P,
                Calibration.DRIVE_MOVE_PID_I,
                Calibration.DRIVE_MOVE_PID_D,
                encoderAvgPIDSource,
                arcadeDrivePIDOutput.move,
                Calibration.DRIVE_PID_SAMPLE_RATE);

        private final PIDController rotController = new PIDController(
                Calibration.DRIVE_ROTATE_PID_P,
                Calibration.DRIVE_ROTATE_PID_I,
                Calibration.DRIVE_ROTATE_PID_D,
                encoderDiffPIDSource,
                arcadeDrivePIDOutput.rotate,
                Calibration.DRIVE_PID_SAMPLE_RATE);

        private final SmartTimer targetTimer = new SmartTimer();
        public final Output<Boolean> onTarget = addOutput("done", false,
            () -> targetTimer.hasReachedTime(Calibration.DRIVE_PID_AFTER_TIMING));

        private final double moveSetpoint;
        private final double rotSetpoint;

        public ArcadeServo (double moveSetpoint, double rotSetpoint) {
            super(Drive.this, ArcadeServo.class);
            this.moveSetpoint = moveSetpoint;
            this.rotSetpoint = rotSetpoint;
        }

        @Override
        protected void begin () {
            resetEncoders();

            moveController.setOutputRange(
                    -Calibration.DRIVE_MOVE_PID_MAX,
                    Calibration.DRIVE_MOVE_PID_MAX
            );
            moveController.setAbsoluteTolerance(Calibration.DRIVE_MOVE_TOLERANCE);
            moveController.setSetpoint(moveSetpoint);

            rotController.setOutputRange(
                    -Calibration.DRIVE_ROTATE_PID_MAX,
                    Calibration.DRIVE_ROTATE_PID_MAX
            );
            rotController.setAbsoluteTolerance(Calibration.DRIVE_ROTATE_TOLERANCE);
            rotController.setSetpoint(rotSetpoint);

            rotController.enable();

            // Stagger the timings of the PIDs slightly
            try {
                // 500 = 1000 / 2
                // Set up PIDs to output in even staggering
                Thread.sleep((long) (Calibration.DRIVE_PID_SAMPLE_RATE*500));
            } catch (InterruptedException e) {
                // Do nothing
            }

            moveController.enable();
        }

        @Override
        protected void run () {
            if (moveController.onTarget() && rotController.onTarget()) {
                targetTimer.startIfNotRunning();
            } else {
                targetTimer.stopAndReset();
            }

            arcadeDrivePIDOutput.update();
        }

        @Override
        protected void end () {
            moveController.reset();
            rotController.reset();

            targetTimer.stopAndReset();
        }
    }

    public Drive () {
        super(Drive.class);

        robotDrive.setDeadband(0.04);

        setDefaultAction(idle);
    }
}
