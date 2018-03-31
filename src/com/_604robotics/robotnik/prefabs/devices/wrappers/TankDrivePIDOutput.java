package com._604robotics.robotnik.prefabs.devices.wrappers;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * PID output for tank drive.
 */
public class TankDrivePIDOutput {
    private final DifferentialDrive drive;
    private final boolean squaredInputs;

    private double leftPower;
    private double rightPower;

    /**
     * Left power PID output.
     */
    public final PIDOutput left = new PIDOutput() {
        @Override
        public void pidWrite (double output) {
            leftPower = output;
        }
    };

    /**
     * Right power PID output.
     */
    public final PIDOutput right = new PIDOutput() {
        @Override
        public void pidWrite (double output) {
            rightPower = output;
        }
    };

    /**
     * Creates a tank drive PID output.
     * @param drive Robot drive to use.
     */
    public TankDrivePIDOutput (DifferentialDrive drive) {
        this(drive, true);
    }

    public TankDrivePIDOutput (DifferentialDrive drive, boolean squaredInputs) {
        this.drive = drive;
        this.squaredInputs = squaredInputs;
    }

    public void update () {
        drive.tankDrive(leftPower, rightPower, squaredInputs);
    }

    public void reset () {
        leftPower = 0;
        rightPower = 0;
        update();
    }
}