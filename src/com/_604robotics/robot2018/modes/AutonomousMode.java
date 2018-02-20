package com._604robotics.robot2018.modes;

import com._604robotics.robot2018.Robot2018;
import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.macros.ArcadeTimedDriveMacro;
import com._604robotics.robot2018.modules.Arm;
import com._604robotics.robot2018.modules.Clamp;
import com._604robotics.robot2018.modules.Drive;
import com._604robotics.robot2018.modules.Elevator;
import com._604robotics.robot2018.modules.Intake;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.coordinators.SleepCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;
import com._604robotics.robotnik.prefabs.flow.Pulse;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import com._604robotics.robotnik.utils.AutonMovement;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousMode extends Coordinator {
    private final Robot2018 robot;

    private final Coordinator rotateLeftStateMacro;
    private final Coordinator rotateRightStateMacro;
    private final Coordinator forwardStateMacro;
    private final Coordinator forwardSwitchMacro;
    private final Coordinator backwardStateMacro;
    private final Coordinator kinematicFallback;

    private Coordinator selectedModeMacro;

    public AutonomousMode (Robot2018 robot) {
        this.robot = robot;

        rotateLeftStateMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_STILL_TARGET,
                Calibration.DRIVE_ROTATE_LEFT_TARGET);
        rotateRightStateMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_STILL_TARGET,
                Calibration.DRIVE_ROTATE_RIGHT_TARGET);
        forwardStateMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_FORWARD_TARGET,
                Calibration.DRIVE_ROTATE_STILL_TARGET);
        forwardSwitchMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_FORWARD_SWITCH_INCHES, 0);
        backwardStateMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_BACKWARD_TARGET,
                Calibration.DRIVE_ROTATE_STILL_TARGET);
        kinematicFallback =new ArcadeTimedDriveMacro(robot) {{
            
        }

        @Override
        protected double getMovePower() {
            return Calibration.DRIVE_MOVE_PID_MAX;
        }

        @Override
        protected double getRotatePower() {
            return 0;
        }

        @Override
        protected double getTime() {
        	// TODO: change to a calibration value
            return 5;
        }};
    }

    @Override
    public void begin () {
        switch (robot.dashboard.autonMode.get()) {
            case ROTATE_LEFT_TEST:
                selectedModeMacro = rotateLeftStateMacro;
                break;
            case ROTATE_RIGHT_TEST:
                selectedModeMacro = rotateRightStateMacro;
                break;
            case FORWARD_6:
                selectedModeMacro = forwardStateMacro;
                break;
            case BACKWARD_6:
                selectedModeMacro = backwardStateMacro;
                break;
            case DEMO_NEW_AUTON:
                selectedModeMacro = new DemoStateMacro();
                break;
            case FORWARD_SWITCH:
                selectedModeMacro = forwardSwitchMacro;
                break;
            case SIDE_LEFT_SWITCH:
                selectedModeMacro = new SideLeftMacro();
                break;
            case APPENDAGE_TEST:
                selectedModeMacro = new ElevatorMove(8000, Calibration.ELEVATOR_PID_TIME_RUN);
                break;
            default:
                selectedModeMacro = null;
                break;
        }

        if (selectedModeMacro != null) {
            selectedModeMacro.start();
        }
    }

    @Override
    public boolean run () {
        if (selectedModeMacro == null) {
            return false;
        }

        return selectedModeMacro.execute();
    }

    @Override
    public void end () {
        if (selectedModeMacro != null) {
            selectedModeMacro.stop();
        }
    }
    
    protected final class ClampExtend extends Coordinator {
    	private Clamp.HoldExtend autonClampExtend;
    	private boolean sent;
    	
    	public ClampExtend() {
    		autonClampExtend = robot.clamp.new HoldExtend();
    		sent = false;
    	}
    	
    	@Override
    	public void begin() {
    		autonClampExtend.activate();
    		sent = false;
    	}
    	
    	@Override
    	public boolean run() {
    		if( sent ) {
    			return false;
    		} else {
    			sent = true;
        		autonClampExtend.activate();
        		return true;
    		}
    	}
    	
    	@Override
    	public void end() {
    		// Do nothing
    	}
    }
    
    protected final class ClampRetract extends Coordinator {
    	private Clamp.HoldRetract autonClampRetract;
    	private boolean sent;
    	
    	public ClampRetract() {
    		autonClampRetract = robot.clamp.new HoldRetract();
    		sent = false;
    	}
    	
    	@Override
    	public void begin() {
    		autonClampRetract.activate();
    		sent = false;
    	}
    	
    	@Override
    	public boolean run() {
    		if( sent ) {
    			return false;
    		} else {
    			sent = true;
        		autonClampRetract.activate();
        		return true;
    		}
    	}
    	
    	@Override
    	public void end() {
    		// Do nothing
    	}
    }
    
    protected final class ElevatorMove extends Coordinator {
    	private Elevator.Setpoint autonElevator;
    	private SmartTimer timeElapsed = new SmartTimer();
    	private double elevatorPosition;
    	private double time;
    	
    	public ElevatorMove(double elevatorPos, double seconds) {
    		elevatorPosition = elevatorPos;
    		time = seconds;
    		autonElevator = robot.elevator.new Setpoint(elevatorPosition);
    	}
    	
    	 @Override
         public void begin() {
             autonElevator.activate();
             timeElapsed.start();
         }
         
         @Override
         public boolean run() {
             autonElevator.activate();
             return !timeElapsed.hasReachedTime(time);
         }
         
         @Override
         public void end() {
             timeElapsed.stopAndReset();
         }
    }
    
    protected final class ArmMove extends Coordinator {
        private Arm.Setpoint autonArm;
        private SmartTimer timeElapsed = new SmartTimer();
        private double armPosition;
        private double time;
        
        public ArmMove(double armPos, double seconds) {
            armPosition = armPos;
            time = seconds;
            autonArm = robot.arm.new Setpoint(armPosition);
        }
        
        @Override
        public void begin() {
            autonArm.activate();
            timeElapsed.start();
        }
        
        @Override
        public boolean run() {
            autonArm.activate();
            return !timeElapsed.hasReachedTime(time);
        }
        
        @Override
        public void end() {
            timeElapsed.stopAndReset();
        }
    }

    protected final class IntakeMove extends Coordinator {
    	private SmartTimer timeElapsed;
    	private Intake.Run autonIntake;
    	private double power;
    	private double time;
    	
    	public IntakeMove(double pow, double seconds) {
    		power = pow;
    		time = seconds;
    		timeElapsed = new SmartTimer();
    		autonIntake = robot.intake.new Run(power);
    	}
    	
    	@Override
    	public void begin() {
    		timeElapsed.start();
    		autonIntake.activate();
    	}
    	
    	@Override
    	public boolean run() {
    		autonIntake.activate();
    		return !timeElapsed.hasReachedTime(time);
    	}
    	
    	@Override
    	public void end() {
    		timeElapsed.stopAndReset();
    	}
    }
    
    protected final class ArcadePIDCoordinator extends Coordinator {
        private Logger arcadePIDLog=new Logger(ArcadePIDCoordinator.class);
        // Timer that is started to continue running PID for some time after equilibrium
        private SmartTimer timeElapsed = new SmartTimer();
        private Drive.ArcadeDrive arcadeDrive = robot.drive.new ArcadeDrive(false);
        private Pulse PIDTargetPulse=new Pulse();
        // PIDOutputs write to persistent Input that retain their values
        // This prevents jerky movement when PIDs don't run often enough
        // Rotation PIDOutput
        private PIDOutput rotateBot = new PIDOutput() {
            @Override
            public synchronized void pidWrite(double output) {
                arcadeDrive.rotatePower.set(output);
            }
        };
        // Move PIDOutput
        private PIDOutput moveBot = new PIDOutput() {
            @Override
            public synchronized void pidWrite(double output) {
                arcadeDrive.movePower.set(output);
            }
        };
        // Encoder rotation PIDSource
        private PIDSource encoderDiff = new PIDSource() {

            private PIDSourceType type;

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {type = pidSource;}

            @Override
            public PIDSourceType getPIDSourceType() {return type;}

            @Override
            public double pidGet() {
                return -robot.drive.rightClicks.get()+robot.drive.leftClicks.get();
            }
        };
        // Encoder moving PIDSource
        private PIDSource encoderAvg = new PIDSource() {

            private PIDSourceType type;

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {type = pidSource;}

            @Override
            public PIDSourceType getPIDSourceType() {return type;}

            @Override
            public double pidGet() {
                return (robot.drive.rightClicks.get()+robot.drive.leftClicks.get())/2;
            }

        };
        // Declaration of PIDControllers
        // Avoid initialization here because parameters require tweaking as well
        private PIDController rotController;
        private PIDController moveController;
        private double moveSetpoint;
        private double rotSetpoint;

        public ArcadePIDCoordinator(double moveSetpoint, double rotSetpoint) {
            super();
            this.moveSetpoint = moveSetpoint;
            this.rotSetpoint = rotSetpoint;
        }

        public double getMoveError() {
            return moveController.getError();
        }

        public double getRotError() {
            return rotController.getError();
        }

        @Override
        protected void begin() {
            // Activate arcadeDrive and reset encoder and gyro
            arcadeDrive.activate();
            robot.drive.resetSensors();
            // Set up PIDSource details
            encoderDiff.setPIDSourceType(PIDSourceType.kDisplacement);
            encoderAvg.setPIDSourceType(PIDSourceType.kDisplacement);
            // Set up rotation PID controller
            rotController = new PIDController(Calibration.DRIVE_ROTATE_PID_P,
                    Calibration.DRIVE_ROTATE_PID_I,
                    Calibration.DRIVE_ROTATE_PID_D,
                    encoderDiff,
                    rotateBot,
                    Calibration.DRIVE_PID_SAMPLE_RATE);
            rotController.setSetpoint(rotSetpoint);
            rotController.setOutputRange(-Calibration.DRIVE_ROTATE_PID_MAX,
                    Calibration.DRIVE_ROTATE_PID_MAX);
            rotController.setAbsoluteTolerance(Calibration.DRIVE_ROTATE_TOLERANCE);
            // Set up move PID controller
            moveController = new PIDController(Calibration.DRIVE_MOVE_PID_P,
                    Calibration.DRIVE_MOVE_PID_I,
                    Calibration.DRIVE_MOVE_PID_D,
                    encoderAvg,
                    moveBot,
                    Calibration.DRIVE_PID_SAMPLE_RATE);
            moveController.setSetpoint(moveSetpoint);
            moveController.setOutputRange(-Calibration.DRIVE_MOVE_PID_MAX,
                    Calibration.DRIVE_MOVE_PID_MAX);
            moveController.setAbsoluteTolerance(Calibration.DRIVE_MOVE_TOLERANCE);
            arcadePIDLog.log("INFO", "Enabling rotation controller");
            rotController.enable();
            // Stagger the timings of the PIDs slightly
            try {
                // 500 = 1000 / 2
                // Set up PIDs to output in even staggering
                Thread.sleep((long) (Calibration.DRIVE_PID_SAMPLE_RATE*500));
            } catch (InterruptedException e) {
                // Do nothing
            }
            arcadePIDLog.log("INFO", "Enabling move controller");
            moveController.enable();
            // Instead of using complex logic to start timer,
            // Start timer here and constantly reset until setpoint is reached
            timeElapsed.start();
            PIDTargetPulse.update(true);
        }

        @Override
        protected synchronized boolean run() {
            arcadeDrive.activate();
            //System.out.println("Move error is " + getMoveError() + ", Rot error is " + getRotError());
            return timeElapsed.runUntil(Calibration.DRIVE_PID_AFTER_TIMING, new Runnable() {
                @Override
                public void run() {
                    boolean targetReached = rotController.onTarget() && moveController.onTarget();
                    if (!targetReached) {
                        timeElapsed.reset();
                        PIDTargetPulse.update(true);
                    } else {
                        PIDTargetPulse.update(false);
                    }
                    if (PIDTargetPulse.isFallingEdge()) {
                        arcadePIDLog.log("INFO", "Target reached, enabling timer");
                    } else if (PIDTargetPulse.isRisingEdge()) {
                        arcadePIDLog.log("INFO", "Target left, disabling timer");
                    }
                }
            });
        }

        @Override
        protected void end() {
            arcadePIDLog.log("INFO","Final Rotate error is "+rotController.getError());
            arcadePIDLog.log("INFO","Final Move error is "+moveController.getError());
            rotController.disable();
            moveController.disable();
            timeElapsed.stopAndReset();
        }
    }

    private class ArcadePIDStateMacro extends StatefulCoordinator {
        private Logger arcadePIDLog=new Logger("ArcadePIDStateMacro");
        private ArcadePIDCoordinator controller;
        public ArcadePIDStateMacro (double moveSetpoint, double rotSetpoint) {
            super(ArcadePIDStateMacro.class);
            arcadePIDLog.log("INFO", "Move Setpoint is "+moveSetpoint);
            arcadePIDLog.log("INFO", "Rotate setpoint is "+rotSetpoint);
            // Set up a rotate class with both Move and Rotate PIDs
            // Instead of just setting a difference, actively move forwards/backwards to compensate for REAL life
            // This avoids COMPLEX imperfections and reduces many issues to be purely IMAGINARY
            controller = new ArcadePIDCoordinator(moveSetpoint, rotSetpoint);
            addState("ArcadePID",controller);
        }
    }
    
    private class IntakeMacro extends StatefulCoordinator {
        public IntakeMacro() {
            super(IntakeMacro.class);
            addState("Intake cube", new IntakeMove(0.5,1));
            addState("Sleep 0.25 seconds", new SleepCoordinator(0.25));
            addState("Clamp cube", new ClampRetract());
        }
    }

    private class DemoStateMacro extends StatefulCoordinator {
        public DemoStateMacro() {
            super(DemoStateMacro.class);
            addStates(new IntakeMacro());
            addState("Forward 12 feet", new ArcadePIDCoordinator(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 12*12+1), 0));
            //addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
            addState("Rotate 180 right", new ArcadePIDCoordinator(0,AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 180)));
            //addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
            addState("Forward 12 feet", new ArcadePIDCoordinator(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 12*12+1), 0));
        }
    }
    
    private class SideLeftMacro extends StatefulCoordinator {
        public SideLeftMacro() {
            super(SideLeftMacro.class);
            addState("Backward 210 inches", new ArcadePIDCoordinator(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -210), 0));
            //addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
            addState("Rotate 90 right", new ArcadePIDCoordinator(0,AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 90)));
            //addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
            addState("Backward 69 inches", new ArcadePIDCoordinator(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -69), 0));
            addState("Rotate 90 left", new ArcadePIDCoordinator(0,AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -90)));
            addState("Forward 14 inches", new ArcadePIDCoordinator(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 14), 0));
        }
    }
}