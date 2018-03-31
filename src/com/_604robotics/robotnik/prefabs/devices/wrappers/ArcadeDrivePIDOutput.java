package com._604robotics.robotnik.prefabs.devices.wrappers;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

/**
 * PID output for arcade drive.
 */
public class ArcadeDrivePIDOutput {
    private final DifferentialDrive drive;
    private final boolean squaredInputs;

    private double movePower;
    private double rotatePower;

    /**
     * Manual power PID output.
     */
    public final PIDOutput move = new PIDOutput() {
        @Override
        public void pidWrite (double output) {
            movePower = output;
        }
    };

    /**
     * Rotate power PID output.
     */
    public final PIDOutput rotate = new PIDOutput() {
        @Override
        public void pidWrite (double output) {
            rotatePower = output;
        }
    };

    /**
     * Creates an arcade drive PID output.
     * @param drive Robot drive to use.
     */
    public ArcadeDrivePIDOutput (DifferentialDrive drive) {
        this(drive, true);
    }

    public ArcadeDrivePIDOutput (DifferentialDrive drive, boolean squaredInputs) {
        this.drive = drive;
        this.squaredInputs = squaredInputs;
    }

    public void update () {
        drive.arcadeDrive(movePower, rotatePower, squaredInputs);
    }

    public void reset () {
        movePower = 0;
        rotatePower = 0;
        update();
    }
}