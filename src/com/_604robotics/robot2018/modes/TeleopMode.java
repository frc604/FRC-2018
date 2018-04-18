package com._604robotics.robot2018.modes;

import com._604robotics.marionette.InputPlayer;
import com._604robotics.marionette.InputRecorder;
import com._604robotics.marionette.InputRecording;
import com._604robotics.marionette.MarionetteJoystick;
import com._604robotics.robot2018.Robot2018;
import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.modules.Arm;
import com._604robotics.robot2018.modules.Clamp;
import com._604robotics.robot2018.modules.Drive;
import com._604robotics.robot2018.modules.Elevator;
import com._604robotics.robot2018.modules.Intake;
import com._604robotics.robot2018.modules.Intake.Passive;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.flow.Pulse;
import com._604robotics.robotnik.prefabs.flow.Toggle;
import com._604robotics.robotnik.prefabs.inputcontroller.xbox.XboxController;

import java.io.IOException;

public class TeleopMode extends Coordinator {

    private static final Logger logger = new Logger(TeleopMode.class);

    private final InputPlayer inputPlayer = new InputPlayer();
    private InputRecorder inputRecorder;

    private final MarionetteJoystick driverJoystick = new MarionetteJoystick(0, inputPlayer, 0);
    private final MarionetteJoystick manipJoystick = new MarionetteJoystick(1, inputPlayer, 1);

    private final XboxController driver = new XboxController(driverJoystick);
    private final XboxController manip = new XboxController(manipJoystick);

    private final Robot2018 robot;

    private final DriveManager driveManager;
    private final ElevatorManager elevatorManager;
    private final IntakeManager intakeManager;
    private final ArmManager armManager;
    private final ClampManager clampManager;
    private final Logger test = new Logger("Teleop");

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
        elevatorManager = new ElevatorManager();
        intakeManager = new IntakeManager();
        armManager = new ArmManager();
        clampManager = new ClampManager();
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

    public void startPlayback (InputRecording recording) {
        inputPlayer.startPlayback(recording);
    }

    public void stopPlayback () {
        inputPlayer.stopPlayback();
    }

    @Override
    protected void begin () {
        if (inputPlayer.isPlaying()) {
            logger.info("Playing back Marionette recording");
        } else if (robot.dashboard.recordAuton.get()) {
            logger.info("Recording inputs with Marionette");
            inputRecorder = new InputRecorder(1200, driverJoystick, manipJoystick);
        }
    }

    @Override
    protected boolean run () {
    	updateControls();
        process();
        return true;
    }

    @Override
    protected void end () {
        if (inputRecorder != null) {
            final InputRecorder oldInputRecorder = inputRecorder;
            inputRecorder = null;

            try {
                logger.info("Terminating Marionette recording");
                oldInputRecorder.close();

                final String fileName = robot.dashboard.recordAutonFile.get();
                logger.info("Saving Marionette recording to \"" + fileName + "\"");
                oldInputRecorder.getRecording().save("/home/lvuser/" + robot.dashboard.recordAutonFile.get());
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private void updateControls() {
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
    
    private void process() {
    	driveManager.run();
        intakeManager.run();
        armManager.run();
        elevatorManager.run();
        clampManager.run();
    }
    
    private class DriveManager {
        private final Drive.ArcadeDrive arcade;
        private final Drive.TankDrive tank;
        private final Drive.Idle idle;
        private CurrentDrive currentDrive;
        private Toggle inverted;
        private Toggle gearState;

        public DriveManager () {
            idle=robot.drive.new Idle();
            arcade=robot.drive.new ArcadeDrive();
            tank=robot.drive.new TankDrive();
            // TODO: Expose on dashboard
            currentDrive=CurrentDrive.ARCADE;
            // TODO: Expose on dashboard
            inverted=new Toggle(false);
            gearState=new Toggle(false);
        }

        public void run() {
        	double leftY = driver.leftStick.y.get();
        	double rightY = driver.rightStick.y.get();
        	double rightX = driver.rightStick.x.get();
            // Set gears
            gearState.update(driverRightBumper);
            // Will probably be double solenoid but waiting
            if (gearState.isInOnState()) {
                robot.shifter.highGear.activate();
            } else if (gearState.isInOffState()) {
                robot.shifter.lowGear.activate();
            }
            // Flip values if xbox inverted
            inverted.update(driverLeftBumper);
            robot.dashboard.XboxFlipped.set(inverted.isInOnState());
            if (inverted.isInOnState()) {
                leftY*=-1;
                rightY*=-1;
            }
            // Get Dashboard option for drive
            switch (robot.dashboard.driveMode.get()){
                case OFF:
                    currentDrive=CurrentDrive.IDLE;
                    break;
                case ARCADE:
                    currentDrive=CurrentDrive.ARCADE;
                    break;
                case TANK:
                    currentDrive=CurrentDrive.TANK;
                    break;
                case DYNAMIC:
                    // Dynamic Drive mode detection logic
                    if (currentDrive == CurrentDrive.TANK) {
                        if (Math.abs(rightY) <= 0.2 && Math.abs(rightX) > 0.3) {
                            currentDrive = CurrentDrive.ARCADE;
                        }
                    } else { // currentDrive == CurrentDrive.ARCADE
                        if (Math.abs(rightX) <= 0.2 && Math.abs(rightY) > 0.3) {
                            currentDrive = CurrentDrive.TANK;
                        }
                    }
                    break;
                default:
                    System.out.println("This should never happen!");
                    System.out.println("Current value is:"+robot.dashboard.driveMode.get());
            }

            // Set appropriate drive mode depending on dashboard option
            switch (currentDrive) {
                case IDLE:
                    idle.activate();
                    break;
                case ARCADE:
                    arcade.movePower.set(leftY);
                    arcade.rotatePower.set(rightX);
                    arcade.activate();
                    break;
                case TANK:
                    tank.leftPower.set(leftY);
                    tank.rightPower.set(rightY);
                    tank.activate();
                    break;
            }
        }
    }
    
    private boolean clampingOverride = false;
    
    private class IntakeManager {
    	private final Intake.Idle idle;
    	private final Intake.Run run;
    	private final Intake.Passive passive;
    	
    	public IntakeManager() {
    		idle = robot.intake.new Idle();
    		run = robot.intake.new Run();
    		passive = robot.intake.new Passive();
    	}
    	
    	public void run() {
    		if( driverLeftTrigger != 0 || driverRightTrigger != 0 ) {
    			double output = 0;
    			if( driverRightTrigger >= driverLeftTrigger ) {
    				output = (driverRightTrigger*driverRightTrigger);
    			} else if( driverLeftTrigger > driverRightTrigger ) {
    			    if( driverDPad ) {
    			        output = -Calibration.INTAKE_OUTAKE_MANIPULATOR_OVERDRIVE_MODIFIER*Math.sqrt(driverLeftTrigger);
    			    } else {
    			        output = -Calibration.INTAKE_OUTAKE_DRIVER_MODIFIER*Math.sqrt(driverLeftTrigger);
    			    }
    			}
    			run.runPower.set(output);
    			run.activate();
    		} else if( manipRightBumper || manipRightTrigger != 0 ) {
    			double output = 0;
    			if( manipRightTrigger != 0 ) {
    			    if( manipDPad ) {
                        output = -Calibration.INTAKE_OUTAKE_MANIPULATOR_OVERDRIVE_MODIFIER;
                    } else {
                        output = -Calibration.INTAKE_OUTAKE_MANIPULATOR_MODIFIER*Math.sqrt(manipRightTrigger);
                    }
                } else if( manipRightBumper ) {
                    output = 1;
                }
    		    run.runPower.set(output);
    		    run.activate();
    		} else if(clampingOverride) {
    		    passive.activate();
    		} else {
    			idle.activate();
    		}
    	}
    }
    
    private class ClampManager {
    	private final Clamp.Extend extend;
    	private final Clamp.Retract retract;
    	private final Toggle clamping;
    	
    	public ClampManager() {
    		extend = robot.clamp.new Extend();
    		retract = robot.clamp.new Retract();
    		clamping = new Toggle(true);
    	}
    	
    	public void run() {
    		clamping.update(manipX);
            if (clamping.isInOnState()) {
                retract.activate();
                clampingOverride = true;
            } else if (clamping.isInOffState()) {
                extend.activate();
                clampingOverride = false;
            }
    	}
    }
    
    private boolean elevatorOverride = false;
    
    private class ElevatorManager {
        private final Elevator.Move move;
        private final Elevator.Setpoint setpoint;
        private double holdSetpoint;

        public ElevatorManager() {
            move = robot.elevator.new Move();
            setpoint = robot.elevator.new Setpoint();
        }

        public void run() {
            // Only use when absolutely necessary
        	if( manipBack ) {
        		 robot.elevator.encoder.zero();
                 setpoint.target_clicks.set(robot.elevator.encoderClicks.get());
                 setpoint.activate();
        	} 
        	if( Calibration.TANDEM_ACTIVE && manipLeftTrigger == 0 && elevatorOverride ) {
        	    System.out.println("Warning: overriding elevator");
        		setpoint.target_clicks.set(Calibration.ELEVATOR_RAISE_TARGET);
        		setpoint.activate();
        	} else if( manipLeftJoystickY != 0 ) {
        	    // Scale negative power for safety
        	    double elevPower = manipLeftJoystickY;
        	    if (elevPower<0) {
        	        elevPower*=0.5;
        	    }
        		move.liftPower.set(elevPower);
        		move.activate();
        	} else if( manipA ) {
        		setpoint.target_clicks.set(Calibration.ELEVATOR_LOW_TARGET);
        		setpoint.activate();
        	} else if( manipB && manipLeftBumper ) {
        		setpoint.target_clicks.set(Calibration.ELEVATOR_MID_TARGET);
        		setpoint.activate();
        	} else if( manipY && manipLeftBumper ) {
        		setpoint.target_clicks.set(Calibration.ELEVATOR_HIGH_TARGET);
        		setpoint.activate();
        	} else if( driverA ) {
        		setpoint.target_clicks.set(Calibration.ELEVATOR_LOW_TARGET);
        		setpoint.activate();
        	} else if( driverB && driverLeftJoystickButton ) {
        		setpoint.target_clicks.set(Calibration.ELEVATOR_MID_TARGET);
        		setpoint.activate();
        	} else if( driverY && driverLeftJoystickButton ) {
        		setpoint.target_clicks.set(Calibration.ELEVATOR_HIGH_TARGET);
        		setpoint.activate();
            } else {
                if (Calibration.ELEVATOR_HOLD_ACTIVE) {
                    // This should only be called once
                    if (robot.elevator.getHoldElevatorClicks) {
                        holdSetpoint = robot.elevator.encoderClicks.get();
                        robot.elevator.resetIntegral(Calibration.ELEVATOR_RESET_SUM);
                        robot.elevator.getHoldElevatorClicks = false;
                    }
                    setpoint.target_clicks.set(holdSetpoint);
                    setpoint.activate();
                } else {
                    move.liftPower.set(0.0);
                    move.activate();
                }
            }
        }
    }
    
    private enum CurrentDrive {
        IDLE, ARCADE, TANK
    }

    private class ArmManager {
    	private final Arm.Move move;
    	private final Arm.Setpoint setpoint;
    	private double holdSetpoint = Calibration.ARM_LOW_TARGET;
    	private final Pulse bottomPulse = new Pulse();
    	
    	public ArmManager() {
    		move = robot.arm.new Move();
    		setpoint = robot.arm.new Setpoint(Calibration.ARM_LOW_TARGET);
    		bottomPulse.update(false);
    	}
    	
    	public void run() {
    	    bottomPulse.update(robot.arm.getBottomLimit());
    	    if (manipStart || bottomPulse.isRisingEdge()) {
    	        robot.arm.encoder.zero(Calibration.ARM_BOTTOM_LOCATION);
    	        getHoldArmClicks = true; // Need to get hold setpoint again
    	    }
    		if( manipRightJoystickY != 0 ) {
    			if( Calibration.TANDEM_ACTIVE && manipLeftTrigger == 0 && robot.elevator.encoder.getPosition() < Calibration.ELEVATOR_BUMPER_CLEAR && 
        				robot.arm.encoder.getPosition() < Calibration.ARM_RAISE_TARGET  && 
        				manipRightJoystickY > 0 ) {
    			    
        			elevatorOverride = true;
        			setpoint.target_clicks.set(holdSetpoint);
                    setpoint.activate();
        		} else {
        			elevatorOverride = false;
        			double motorPower = manipRightJoystickY;
        			if (motorPower<0 && !manipRightJoystickButton) {
        			    motorPower*=0.6;
        			}
        	        // Calculate cosine for torque factor
        	        double angle = robot.arm.encoder.getPosition();
        	        // Cosine is periodic so sawtooth wraparound is not a concern
        	        angle/=Calibration.ARM_ENCODER_FULL_ROT;
        	        angle*=(2*Math.PI);
        	        double cosine = Math.cos(angle);
        	        if (robot.arm.encoderClicks.get()<4700 && robot.arm.encoderClicks.get()> -2500
        	                && motorPower<0) {
        	            motorPower+=Calibration.ARM_F*cosine;
        	        }
        			move.liftPower.set(motorPower);
        			move.activate();
        		}
    			getHoldArmClicks = true;
    		} else if( manipA ) {
    			elevatorOverride = false;
    			setpoint.target_clicks.set(Calibration.ARM_LOW_TARGET);
    			setpoint.activate();
    			getHoldArmClicks = true;
    		} else if( manipB ) {
    		    if (manipLeftBumper) {
    		    	if( Calibration.TANDEM_ACTIVE && manipLeftTrigger == 0 && robot.elevator.encoder.getPosition() < Calibration.ELEVATOR_BUMPER_CLEAR && 
            				robot.arm.encoder.getPosition() < Calibration.ARM_RAISE_TARGET  && 
            				robot.arm.encoder.getPosition() < Calibration.ARM_HIGH_TARGET ) {
    		    	    
            			elevatorOverride = true;
            			setpoint.target_clicks.set(holdSetpoint);
                        setpoint.activate();
            		} else {
            			elevatorOverride = false;
            			setpoint.target_clicks.set(Calibration.ARM_HIGH_TARGET);
            			setpoint.activate();
            		}
    		    } else {
    		    	if( Calibration.TANDEM_ACTIVE && manipLeftTrigger == 0 && robot.elevator.encoder.getPosition() < Calibration.ELEVATOR_BUMPER_CLEAR && 
            				robot.arm.encoder.getPosition() < Calibration.ARM_RAISE_TARGET  && 
            				robot.arm.encoder.getPosition() < Calibration.ARM_MID_TARGET ) {
    		    	    
            			elevatorOverride = true;
            			setpoint.target_clicks.set(holdSetpoint);
                        setpoint.activate();
            		} else {
            			elevatorOverride = false;
            			setpoint.target_clicks.set(Calibration.ARM_MID_TARGET);
            			setpoint.activate();
            		}
    		    }
    			getHoldArmClicks = true;
    		} else if( manipY ) {
    			if( Calibration.TANDEM_ACTIVE && manipLeftTrigger == 0 && robot.elevator.encoder.getPosition() < Calibration.ELEVATOR_BUMPER_CLEAR && 
        				robot.arm.encoder.getPosition() < Calibration.ARM_RAISE_TARGET  && 
        				robot.arm.encoder.getPosition() < Calibration.ARM_HIGH_TARGET ) {
    			    System.out.println("Warning: activating tandem");
        			elevatorOverride = true;
        			setpoint.target_clicks.set(holdSetpoint);
                    setpoint.activate();
        		} else {
        			elevatorOverride = false;
        			setpoint.target_clicks.set(Calibration.ARM_HIGH_TARGET);
        			setpoint.activate();
        		}
    			getHoldArmClicks = true;
    		} else if( driverA ) {
    			elevatorOverride = false;
    			setpoint.target_clicks.set(Calibration.ARM_LOW_TARGET);
    			setpoint.activate();
    			getHoldArmClicks = true;
    		} else if( driverB ) {
                if (driverLeftJoystickButton) {
                	if( Calibration.TANDEM_ACTIVE && manipLeftTrigger == 0 && robot.elevator.encoder.getPosition() < Calibration.ELEVATOR_BUMPER_CLEAR && 
            				robot.arm.encoder.getPosition() < Calibration.ARM_RAISE_TARGET  && 
            				robot.arm.encoder.getPosition() < Calibration.ARM_HIGH_TARGET ) {
            			elevatorOverride = true;
            			setpoint.target_clicks.set(holdSetpoint);
                        setpoint.activate();
            		} else {
            			elevatorOverride = false;
            			setpoint.target_clicks.set(Calibration.ARM_HIGH_TARGET);
            			setpoint.activate();
            		}
                } else {
                	if( Calibration.TANDEM_ACTIVE && manipLeftTrigger == 0 && robot.elevator.encoder.getPosition() < Calibration.ELEVATOR_BUMPER_CLEAR && 
            				robot.arm.encoder.getPosition() < Calibration.ARM_RAISE_TARGET  && 
            				robot.arm.encoder.getPosition() < Calibration.ARM_MID_TARGET ) {
            			elevatorOverride = true;
            			setpoint.target_clicks.set(holdSetpoint);
                        setpoint.activate();
            		} else {
            			elevatorOverride = false;
            			setpoint.target_clicks.set(Calibration.ARM_MID_TARGET);
            			setpoint.activate();
            		}
                }
    			getHoldArmClicks = true;
    		} else if( driverY ) {
    			if( Calibration.TANDEM_ACTIVE && manipLeftTrigger == 0 && robot.elevator.encoder.getPosition() < Calibration.ELEVATOR_BUMPER_CLEAR && 
        				robot.arm.encoder.getPosition() < Calibration.ARM_RAISE_TARGET  && 
        				robot.arm.encoder.getPosition() < Calibration.ARM_HIGH_TARGET ) {
        			elevatorOverride = true;
        			setpoint.target_clicks.set(holdSetpoint);
                    setpoint.activate();
        		} else {
        			elevatorOverride = false;
        			setpoint.target_clicks.set(Calibration.ARM_HIGH_TARGET);
        			setpoint.activate();
        		}
    			getHoldArmClicks = true;
            } else {
                // This should only be called once
                if (Calibration.ARM_HOLD_ACTIVE) {
                    if (getHoldArmClicks) {
                        holdSetpoint = robot.arm.encoderClicks.get();
                        robot.arm.resetIntegral(Calibration.ARM_RESET_SUM);
                        getHoldArmClicks = false;
                    }
                    elevatorOverride = false;
                    setpoint.target_clicks.set(holdSetpoint);
                    setpoint.activate();
                } else {
                    move.liftPower.set(0.0);
                    move.activate();
                }
            }
        }
    }
}
