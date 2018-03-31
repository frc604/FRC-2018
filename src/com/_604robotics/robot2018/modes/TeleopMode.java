package com._604robotics.robot2018.modes;

import com._604robotics.robot2018.Robot2018;
import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.modules.Arm;
import com._604robotics.robot2018.modules.Drive;
import com._604robotics.robot2018.modules.Elevator;
import com._604robotics.robot2018.modules.Intake;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.flow.Toggle;
import com._604robotics.robotnik.prefabs.inputcontroller.xbox.XboxController;

public class TeleopMode extends Coordinator {
    private static final Logger logger = new Logger(TeleopMode.class);

    private final XboxController driver = new XboxController(0);
    private final XboxController manip = new XboxController(1);

    private final Robot2018 robot;

    private final DriveManager driveManager;
    private final IntakeManager intakeManager;
    private final ClampManager clampManager;
    private final ElevatorArmManager elevatorArmManager;

    public TeleopMode (Robot2018 robot) {
        driver.leftStick.x.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);
        driver.leftStick.y.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);

        driver.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
        driver.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

        driver.rightStick.x.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);
        driver.rightStick.y.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);

        //driver.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
        driver.rightStick.x.setFactor(1); // WEIRD_WHY_?FES:RLJTH *ROHT guirg
        driver.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);

        manip.leftStick.x.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);
        manip.leftStick.y.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);

        manip.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
        manip.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

        manip.rightStick.x.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);
        manip.rightStick.y.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);

        manip.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
        manip.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);

        this.robot = robot;

        driveManager = new DriveManager();
        intakeManager = new IntakeManager();
        clampManager = new ClampManager();
        elevatorArmManager = new ElevatorArmManager();
    }
    
    private boolean getHoldArmClicks = false;
    
    private double driverLeftJoystickY = 0.0;
    private double driverLeftJoystickX = 0.0;
    private double driverLeftTrigger = 0.0;
    
    private boolean driverLeftJoystickButton = false;
    private boolean driverLeftTriggerButton = false;
    private boolean driverLeftBumper = false;
    
    private double driverRightJoystickY = 0.0;
    private double driverRightJoystickX = 0.0;
    private double driverRightTrigger = 0.0;
    
    private boolean driverRightJoystickButton = false;
    private boolean driverRightTriggerButton = false;
    private boolean driverRightBumper = false;
    
    private boolean driverBack = false;
    private boolean driverStart = false;
    private boolean driverA = false;
    private boolean driverB = false;
    private boolean driverX = false;
    private boolean driverY = false;
    
    private boolean driverDPad = false;
    
    private double manipLeftJoystickY = 0.0;
    private double manipLeftJoystickX = 0.0;
    private double manipLeftTrigger = 0.0;
    
    private boolean manipLeftJoystickButton = false;
    private boolean manipLeftTriggerButton = false;
    private boolean manipLeftBumper = false;
    
    private double manipRightJoystickY = 0.0;
    private double manipRightJoystickX = 0.0;
    private double manipRightTrigger = 0.0;
    
    private boolean manipRightJoystickButton= false;
    private boolean manipRightTriggerButton= false;
    private boolean manipRightBumper= false;
    
    private boolean manipBack= false;
    private boolean manipStart= false;
    private boolean manipA= false;
    private boolean manipB= false;
    private boolean manipX= false;
    private boolean manipY= false;

    private boolean manipDPad = false;

    @Override
    protected void begin () {
        driveManager.start();
        intakeManager.start();
        clampManager.start();
        elevatorArmManager.start();
    }

    @Override
    public boolean run () {
    	updateControls();

        driveManager.execute();
        intakeManager.execute();
        clampManager.execute();
        elevatorArmManager.execute();

        return true;
    }

    @Override
    protected void end () {
        driveManager.stop();
        intakeManager.stop();
        clampManager.stop();
        elevatorArmManager.stop();
    }

    private void updateControls () {
    	driverLeftJoystickY = driver.leftStick.y.get();
        driverLeftJoystickX = driver.leftStick.x.get();
        driverLeftTrigger = driver.triggers.left.get();
        
        driverLeftJoystickButton = driver.buttons.leftStick.get();
        driverLeftTriggerButton = driver.buttons.lt.get();
        driverLeftBumper = driver.buttons.lb.get();
        
        driverRightJoystickY = driver.rightStick.y.get();
        driverRightJoystickX = driver.rightStick.x.get();
        driverRightTrigger = driver.triggers.right.get();
        
        driverRightJoystickButton = driver.buttons.rightStick.get();
        driverRightTriggerButton = driver.buttons.rt.get();
        driverRightBumper = driver.buttons.rb.get();
        
        driverBack = driver.buttons.back.get();
        driverStart = driver.buttons.start.get();
        driverA = driver.buttons.a.get();
        driverB = driver.buttons.b.get();
        driverX = driver.buttons.x.get();
        driverY = driver.buttons.y.get();
        
        driverDPad = driver.dpad.pressed.get();
        
        manipLeftJoystickY = manip.leftStick.y.get();
        manipLeftJoystickX = manip.leftStick.x.get();
        manipLeftTrigger = manip.triggers.left.get();
        
        manipLeftJoystickButton = manip.buttons.leftStick.get();
        manipLeftTriggerButton = manip.buttons.lt.get();
        manipLeftBumper = manip.buttons.lb.get();
        
        manipRightJoystickY = manip.rightStick.y.get();
        manipRightJoystickX = manip.rightStick.x.get();
        manipRightTrigger = manip.triggers.right.get();
        
        manipRightJoystickButton = manip.buttons.rightStick.get();
        manipRightTriggerButton = manip.buttons.rt.get();
        manipRightBumper = manip.buttons.rb.get();
        
        manipBack = manip.buttons.back.get();
        manipStart = manip.buttons.start.get();
        manipA = manip.buttons.a.get();
        manipB = manip.buttons.b.get();
        manipX = manip.buttons.x.get();
        manipY = manip.buttons.y.get();
        
        manipDPad = manip.dpad.pressed.get();
        /*System.out.println("driverLeftY: "+driverLeftJoystickY+", driverRightX"+driverRightJoystickX
                +", driverRightY"+driverRightJoystickY);*/
    }
    
    private enum DriveMode {
        IDLE, ARCADE, TANK
    }

    private class DriveManager extends Coordinator {
        private final Drive.ArcadeDrive arcadeDrive = robot.drive.new ArcadeDrive();
        private final Drive.TankDrive tankDrive = robot.drive.new TankDrive();

        private final Toggle invertedToggle = new Toggle(false);
        private final Toggle highGearToggle = new Toggle(false);

        private DriveMode currentDriveMode;

        @Override
        protected void begin () {
            currentDriveMode = DriveMode.ARCADE;
        }

        @Override
        protected boolean run () {
            double leftY = driverLeftJoystickY;
            double rightX = driverRightJoystickX;
            double rightY = driverRightJoystickY;

            // Set gears
            highGearToggle.update(driverRightBumper);
            if (highGearToggle.isInOnState()) {
                robot.shifter.highGear.activate();
            }

            // Flip values if xbox inverted
            invertedToggle.update(driverLeftBumper);
            robot.dashboard.xboxFlipped.set(invertedToggle.isInOnState());
            if (invertedToggle.isInOnState()) {
                leftY *= -1;
                rightY *= -1;
            }

            // Get Dashboard option for drive
            switch (robot.dashboard.driveMode.get()){
                case OFF: {
                    currentDriveMode = DriveMode.IDLE;
                    break;
                }
                case ARCADE: {
                    currentDriveMode = DriveMode.ARCADE;
                    break;
                }
                case TANK: {
                    currentDriveMode = DriveMode.TANK;
                    break;
                }
                case DYNAMIC: {
                    // Dynamic Drive mode detection logic
                    if (currentDriveMode == DriveMode.TANK) {
                        if (Math.abs(rightY) <= 0.2 && Math.abs(rightX) > 0.3) {
                            currentDriveMode = DriveMode.ARCADE;
                        }
                    } else { // currentDrive == DriveMode.ARCADE
                        if (Math.abs(rightX) <= 0.2 && Math.abs(rightY) > 0.3) {
                            currentDriveMode = DriveMode.TANK;
                        }
                    }
                    break;
                }
                default: {
                    logger.warn("Unrecognized driveMode! Current value: " + robot.dashboard.driveMode.get());
                    break;
                }
            }

            // Set appropriate drive mode depending on dashboard option
            switch (currentDriveMode) {
                case IDLE: {
                    robot.drive.idle.activate();
                    break;
                }
                case ARCADE: {
                    arcadeDrive.movePower.set(leftY);
                    arcadeDrive.rotatePower.set(rightX);
                    arcadeDrive.activate();
                    break;
                }
                case TANK: {
                    tankDrive.leftPower.set(leftY);
                    tankDrive.rightPower.set(rightY);
                    tankDrive.activate();
                    break;
                }
            }

            return true;
        }
    }

    private class IntakeManager extends Coordinator {
    	private final Intake.Run runIntake;

    	public IntakeManager() {
    		runIntake = robot.intake.new Run();
    	}

    	@Override
    	protected boolean run() {
            double output = 0;

    		if (driverLeftTrigger != 0 || driverRightTrigger != 0) {
    			if (driverRightTrigger >= driverLeftTrigger) {
    				output = driverRightTrigger*driverRightTrigger;
    			} else if (driverLeftTrigger > driverRightTrigger) {
    			    if (driverDPad) {
    			        output = -Calibration.INTAKE_OUTAKE_OVERDRIVE_MODIFIER * driverLeftTrigger;
    			    } else {
    			        output = -Calibration.INTAKE_OUTAKE_MODIFIER * driverLeftTrigger;
    			    }
    			}
    		} else if (manipRightBumper || manipRightTrigger != 0) {
    			if (manipRightTrigger != 0) {
    			    if (manipDPad) {
                        output = -Calibration.INTAKE_OUTAKE_OVERDRIVE_MODIFIER;
                    } else {
                        output = -Calibration.INTAKE_OUTAKE_MODIFIER * manipRightTrigger;
                    }
                } else if (manipRightBumper) {
                    output = 1;
                }
    		} else if (robot.clamp.engage.isRunning()) {
    		    output = Calibration.INTAKE_PASSIVE_POWER;
    		}

            runIntake.power.set(output);
            runIntake.activate();

            return true;
    	}
    }
    
    private class ClampManager extends Coordinator {
    	private final Toggle toggle = new Toggle();

        @Override
        protected void begin () {
            toggle.reset(false);
        }

        @Override
    	protected boolean run () {
    		toggle.update(manipX);

            if (toggle.isInOnState()) {
                robot.clamp.engage.activate();
            } else if (toggle.isInOffState()) {
                robot.clamp.release.activate();
            }

            return true;
    	}
    }

    private class ElevatorArmManager extends Coordinator {
        private final Elevator.Manual elevatorManual = robot.elevator.new Manual();
        private final Arm.Manual armManual = robot.arm.new Manual();

        @Override
        protected boolean run () {
            boolean goingUp = false;

            robot.elevator.hold.activate();
            robot.arm.hold.activate();

            if (manipRightJoystickY != 0) {
                if (manipRightJoystickY > 0) {
                    goingUp = true;
                }

                // Scale negative power for safety
                double elevatorPower = manipLeftJoystickY;
                if (elevatorPower < 0) {
                    elevatorPower *=0.5;
                }

                elevatorManual.power.set(elevatorPower);
                elevatorManual.activate();

                double armPower = manipRightJoystickY;
                if (armPower < 0 && !manipRightJoystickButton) {
                    armPower *= 0.6;
                }

                // Calculate cosine for torque factor
                double angle = robot.arm.encoderClicks.get();

                // Cosine is periodic so sawtooth wraparound is not a concern
                angle /= Calibration.ARM_ENCODER_FULL_ROT;
                angle *= 2*Math.PI;

                double cosine = Math.cos(angle);
                if (robot.arm.encoderClicks.get() < 4700 &&
                        robot.arm.encoderClicks.get() > -2500 &&
                        armPower < 0) {
                    armPower += Calibration.ARM_F * cosine;
                }

                armManual.power.set(armPower);
                armManual.activate();
            } else if (manipA) {
                robot.elevator.low.activate();
                robot.arm.low.activate();
            } else if (manipB) {
                goingUp = true;

                if (manipLeftBumper) {
                    robot.elevator.mid.activate();
                    robot.arm.high.activate();
                } else {
                    robot.arm.mid.activate();
                }
            } else if (manipY) {
                goingUp = true;

                if (manipLeftBumper) {
                    robot.elevator.high.activate();
                }

                robot.arm.high.activate();
            } else if (driverA) {
                robot.elevator.low.activate();
                robot.arm.low.activate();
            } else if (driverB) {
                goingUp = true;

                if (driverLeftJoystickButton) {
                    robot.elevator.mid.activate();
                    robot.arm.high.activate();
                } else {
                    robot.arm.mid.activate();
                }
            } else if (driverY) {
                goingUp = true;

                if (driverLeftJoystickButton) {
                    robot.elevator.high.activate();
                }

                robot.arm.high.activate();
            }

            if (goingUp &&
                    Calibration.TANDEM_ACTIVE && !manipLeftTriggerButton &&
                    !robot.elevator.bumperClear.get() && !robot.arm.raised.get()) {
                robot.elevator.raise.activate();
                robot.arm.hold.activate();
            }

            // Only use when absolutely necessary
            if (manipStart) {
                robot.arm.zero.activate();
            }
            if (manipBack) {
                robot.elevator.zero.activate();
            }

            return true;
        }
    }
}
