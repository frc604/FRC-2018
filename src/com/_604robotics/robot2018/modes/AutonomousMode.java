package com._604robotics.robot2018.modes;

import com._604robotics.robot2018.Robot2018;
import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.macros.ArcadeTimedDriveMacro;
import com._604robotics.robot2018.modules.*;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.coordinators.SleepCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.SwitchCoordinator;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import com._604robotics.robotnik.utils.AutonMovement;
import com._604robotics.robotnik.utils.annotations.Unreal;

import java.util.EnumMap;

public class AutonomousMode extends Coordinator {
    private final Robot2018 robot;

    private final EnumMap<Dashboard.AutonMode, Coordinator> autonModes = new EnumMap<>(Dashboard.AutonMode.class);
    private Coordinator selectedMode;

    public AutonomousMode (Robot2018 robot) {
        this.robot = robot;

        autonModes.put(Dashboard.AutonMode.CENTER_SWITCH, new CenterSwitchMacro());
        autonModes.put(Dashboard.AutonMode.BACKWARD_CENTER_SWITCH, new BackwardsCenterSwitchMacro());

        autonModes.put(Dashboard.AutonMode.LEFT_SWITCH, new LeftSwitchMacro());
        autonModes.put(Dashboard.AutonMode.RIGHT_SWITCH, new RightSwitchMacro());

        autonModes.put(Dashboard.AutonMode.LEFT_SCALE_WITH_CROSS, new LeftScaleMacro());
        autonModes.put(Dashboard.AutonMode.RIGHT_SCALE_WITH_CROSS, new RightScaleMacro());

        autonModes.put(Dashboard.AutonMode.LEFT_SCALE_WITHOUT_CROSS, new LeftScaleSameOnlyMacro());
        autonModes.put(Dashboard.AutonMode.RIGHT_SCALE_WITHOUT_CROSS, new RightScaleSameOnlyMacro());

        autonModes.put(Dashboard.AutonMode.FAILSAFE_FORWARD_12, new FallForwardMacro());
        autonModes.put(Dashboard.AutonMode.FAILSAFE_BACKWARD_12, new FallBackMacro());

        autonModes.put(Dashboard.AutonMode.FORWARD_SWITCH, new DriveArcadeServoMacro(
                Calibration.DRIVE_MOVE_FORWARD_SWITCH_INCHES, 0));

        autonModes.put(Dashboard.AutonMode.NEW_SCALE_BACKWARD_LEFT, new NewScaleBackwardMacroLeft());
        autonModes.put(Dashboard.AutonMode.NEW_SCALE_BACKWARD_RIGHT, new NewScaleBackwardMacroRight());
    }

    @Override
    public void begin () {
        // reset arm encoder (again)
        robot.arm.encoder.zero(Calibration.ARM_BOTTOM_LOCATION);

        selectedMode = autonModes.getOrDefault(robot.dashboard.autonMode.get(), null);

        if (selectedMode != null) {
            selectedMode.start();
        }
    }

    @Override
    public boolean run () {
        if (selectedMode == null) {
            return false;
        }

        return selectedMode.execute();
    }

    @Override
    public void end () {
        if (selectedMode != null) {
            selectedMode.stop();
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
    
    protected final class ArmSetPersistent extends Coordinator {
    	private Arm.SetPersistent autonArmSetPersistent;
    	private double setpoint;
    	private boolean sent;
    	
    	public ArmSetPersistent(double clicks) {
    		setpoint = clicks;
    		autonArmSetPersistent = robot.arm.new SetPersistent(setpoint);
    		sent = false;
    	}
    	
    	@Override
    	public void begin() {
    		autonArmSetPersistent.activate();
    		sent = false;
    	}
    	
    	@Override
    	public boolean run() {
    		if( sent ) {
    			return false;
    		} else {
    			sent = true;
        		autonArmSetPersistent.activate();
        		return true;
    		}
    	}
    	
    	@Override
    	public void end() {
    		// Do nothing
    	}
    	
    }
    
    protected final class ElevatorSetPersistent extends Coordinator {
    	private Elevator.SetPersistent autonElevatorSetPersistent;
    	private double setpoint;
    	private boolean sent;
    	
    	public ElevatorSetPersistent(double clicks) {
    		setpoint = clicks;
    		autonElevatorSetPersistent = robot.elevator.new SetPersistent(setpoint);
    		sent = false;
    	}
    	
    	@Override
    	public void begin() {
    		autonElevatorSetPersistent.activate();
    		sent = false;
    	}
    	
    	@Override
    	public boolean run() {
    		if( sent ) {
    			return false;
    		} else {
    			sent = true;
        		autonElevatorSetPersistent.activate();
        		return true;
    		}
    	}
    	
    	@Override
    	public void end() {
    		// Do nothing
    	}
    	
    }
    
    @Deprecated @Unreal("Moves but does not hold afterward")
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
    
    @Deprecated @Unreal("Moves but does not hold afterward")
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
    
    private class FallBackMacro extends StatefulCoordinator {
        public FallBackMacro() {
            super(FallBackMacro.class);
            addStates(new IntakeMacro());
            addState("Backward 144 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(144+1)), 0));
        }
    }
    
    private class FallForwardMacro extends StatefulCoordinator {
        public FallForwardMacro() {
            super(FallForwardMacro.class);
            addStates(new IntakeMacro());
            addState("Forward 144 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, (144+1)), 0));
        }
    }
    
    private class LeftSwitchMacro extends StatefulCoordinator {
        public LeftSwitchMacro() {
            super(LeftSwitchMacro.class);
            addStates(new IntakeMacro());
            addState("Raise elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_RAISE_TARGET));
            addState("Wait for elevator", new SleepCoordinator(0.3));
            addState("Raise arm", new ArmSetPersistent(Calibration.ARM_MID_TARGET));
            addState("Forward 144 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, (144+1)), 0));
            addState("Rotate 90 right", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 90)));
            addState("Forward 24 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, (24)), 0));
            addState("Switch decision", new LeftSideSwitchDecisionMacro());
        }
    }
    
    private class RightSwitchMacro extends StatefulCoordinator {
        public RightSwitchMacro() {
            super(RightSwitchMacro.class);
            addStates(new IntakeMacro());
            addState("Raise elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_RAISE_TARGET));
            addState("Wait for elevator", new SleepCoordinator(0.3));
            addState("Raise arm", new ArmSetPersistent(Calibration.ARM_MID_TARGET));
            addState("Forward 144 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, (144+1)), 0));
            addState("Rotate 90 left", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -90)));
            addState("Forward 24 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, (24)), 0));
            addState("Switch decision", new RightSideSwitchDecisionMacro());
        }
    }

    private class CenterSwitchMacro extends StatefulCoordinator {
        public CenterSwitchMacro() {
            super(CenterSwitchMacro.class);
            addStates(new IntakeMacro());
            addState("Raise elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_SWITCH_CLEAR));
            addState("Wait for elevator", new SleepCoordinator(0.3));
            addState("Raise arm", new ArmSetPersistent(Calibration.ARM_MID_TARGET));
            // Forward distance between front bumper and scale -76 XX
            addState("Forward 23 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 23),0));
            // Choose based on FMS Game Data
            addState("Switch choosing", new CenterSwitchChooserMacro());
            addState("Forward 23 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 23),0));
            addStates(new SwitchEjectMacro());
        }
    }
    
    // Comment me out
    @Unreal("Experimenting with a backwards jerk to deploy outer intake")
    private class BackwardsCenterSwitchMacro extends StatefulCoordinator {
        public BackwardsCenterSwitchMacro() {
            super(BackwardsCenterSwitchMacro.class);
            addState("Jerk backwards", new TimedMacro(-1, 0, 0.5));
            addState("Rotate 180", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 180)));
            addState("Return backwards", new TimedMacro(0.5, 0, 1.5));
            addStates(new CenterSwitchMacro());
        }
    }
    
    private class TimedMacro extends ArcadeTimedDriveMacro {
        protected double movePower;
        protected double rotatePower;
        protected double time;
        
        public TimedMacro( double movePower, double rotatePower, double time ) {
            super(robot);
            this.movePower = movePower;
            this.rotatePower = rotatePower;
            this.time = time;
        }

        @Override
        protected double getMovePower() {
            return movePower;
        }

        @Override
        protected double getRotatePower() {
            return rotatePower;
        }

        @Override
        protected double getTime() {
            return time;
        }
    }
    
    /* Modular Modes */
    private class CenterSwitchChooserMacro extends SwitchCoordinator {
    	public CenterSwitchChooserMacro() {
    		super(CenterSwitchChooserMacro.class);
    		addDefault(new CenterMacroLeft()); // Lucky randomness guaranteed by coin flip
    		addCase(new String[]{"LLL", "LLR", "LRL", "LRR"}, new CenterMacroLeft());
    		addCase(new String[]{"RLL", "RLR", "RRL", "RRR"}, new CenterMacroRight());
    	}
    }
    
    private class LeftSideSwitchDecisionMacro extends SwitchCoordinator {
        public LeftSideSwitchDecisionMacro() {
            super(LeftSideSwitchDecisionMacro.class);
            addDefault(new LowerMacro());
            addCase(new String[]{"LLL", "LLR", "LRL", "LRR"}, new SwitchEjectMacro());
            addCase(new String[]{"RLL", "RLR", "RRL", "RRR"}, new LowerMacro());
        }
    }
    
    private class RightSideSwitchDecisionMacro extends SwitchCoordinator {
        public RightSideSwitchDecisionMacro() {
            super(RightSideSwitchDecisionMacro.class);
            addDefault(new LowerMacro());
            addCase(new String[]{"LLL", "LLR", "LRL", "LRR"}, new LowerMacro());
            addCase(new String[]{"RLL", "RLR", "RRL", "RRR"}, new SwitchEjectMacro());
        }
    }
    
    private class LeftScaleMacro extends StatefulCoordinator {
        public LeftScaleMacro() {
            super(LeftScaleMacro.class);
            addStates(new IntakeMacro());
            addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
            addState("Backward 222 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(222+1)), 0));
            //addState("Set Arm Persistent", new ArmSetPersistent(Calibration.ARM_BALANCE_TARGET));
            //addState("Sleep 0.2 seconds", new SleepCoordinator(0.2));
            addState("Scale chooser", new LeftScaleChooserMacro());
        }
    }
    
    private class LeftScaleSameOnlyMacro extends StatefulCoordinator {
        public LeftScaleSameOnlyMacro() {
            super(LeftScaleSameOnlyMacro.class);
            addStates(new IntakeMacro());
            addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
            addState("Backward 222 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(222+1)), 0));
            //addState("Set Arm Persistent", new ArmSetPersistent(Calibration.ARM_BALANCE_TARGET));
            //addState("Sleep 0.2 seconds", new SleepCoordinator(0.2));
            addState("Scale chooser", new LeftScaleChooserSameOnlyMacro());
        }
    }
    
    private class RightScaleMacro extends StatefulCoordinator {
        public RightScaleMacro() {
            super(RightScaleMacro.class);
            addStates(new IntakeMacro());
            addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
            addState("Backward 222 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(222+1)), 0));
            //addState("Set Arm Persistent", new ArmSetPersistent(Calibration.ARM_BALANCE_TARGET));
            //addState("Sleep 0.2 seconds", new SleepCoordinator(0.2));
            addState("Scale chooser", new RightScaleChooserMacro());
        }
    }
    
    private class RightScaleSameOnlyMacro extends StatefulCoordinator {
        public RightScaleSameOnlyMacro() {
            super(RightScaleSameOnlyMacro.class);
            addStates(new IntakeMacro());
            addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
            addState("Backward 222 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(222+1)), 0));
            //addState("Set Arm Persistent", new ArmSetPersistent(Calibration.ARM_BALANCE_TARGET));
            //addState("Sleep 0.2 seconds", new SleepCoordinator(0.2));
            addState("Scale chooser", new RightScaleChooserSameOnlyMacro());
        }
    }
    
    private class LeftScaleChooserMacro extends SwitchCoordinator {
    	public LeftScaleChooserMacro() {
    		super(LeftScaleChooserMacro.class);
    		addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new NewScaleBackwardMacroLeft());
    		addCase(new String[]{"LRL", "LRR", "RRL", "RRR"}, new NewScaleOppositeMacroLeft());
    	}
    }
    
    private class LeftScaleChooserSameOnlyMacro extends SwitchCoordinator {
        public LeftScaleChooserSameOnlyMacro() {
            super(LeftScaleChooserSameOnlyMacro.class);
            addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new NewScaleBackwardMacroLeft());
            addCase(new String[]{"LRL", "LRR", "RRL", "RRR"}, new SleepCoordinator(1)); // Do nothing
        }
    }
    
    private class RightScaleChooserMacro extends SwitchCoordinator {
    	public RightScaleChooserMacro() {
    		super(RightScaleChooserMacro.class);
    		addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new NewScaleOppositeMacroRight());
    		addCase(new String[]{"LRL", "LRR", "RRL", "RRR"}, new NewScaleBackwardMacroRight());
    	}
    }
    
    private class RightScaleChooserSameOnlyMacro extends SwitchCoordinator {
        public RightScaleChooserSameOnlyMacro() {
            super(RightScaleChooserSameOnlyMacro.class);
            addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new SleepCoordinator(1)); // Do nothing 
            addCase(new String[]{"LRL", "LRR", "RRL", "RRR"}, new NewScaleBackwardMacroRight());
        }
    }
    
    /* Auton Modes */
    
    private class CenterMacroLeft extends StatefulCoordinator {
        public CenterMacroLeft() {
            super(CenterMacroLeft.class);
            addState("Rotate 45 left", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -45)));
            addState("Forward 76+15 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 76+15+1),0));
            addState("Rotate 45 right", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 45)));
            //addState("Forward 19 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 19+1),0));
        }
    }
    
    private class CenterMacroRight extends StatefulCoordinator {
        public CenterMacroRight() {
            super(CenterMacroRight.class);
            addState("Rotate 45 right", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 45)));
            addState("Forward 76 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 76+1),0));
            addState("Rotate 45 left", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -45)));
        }
    }
    
    private class NewScaleBackwardMacroLeft extends StatefulCoordinator {
    	public NewScaleBackwardMacroLeft() {
    		super(NewScaleBackwardMacroLeft.class);
    		addStates(new IntakeMacro());
    		//addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
            addState("Backward 36 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(36+1)), 0));
            //addState("Set Arm Persistent", new ArmSetPersistent(Calibration.ARM_BALANCE_TARGET));
            //addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
            addState("Rotate 35 right", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 35)));
            addState("Backward 9 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(9)), 0));
            addState("Set Arm High Persistent", new ArmSetPersistent(Calibration.ARM_HIGH_TARGET));
            addState("Sleep 1.3 seconds", new SleepCoordinator(1.3));
            addState("Eject cube", new IntakeMove(-0.5,0.5));
            addState("Retract arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
            addState("Retract elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
            addState("Unclamp", new ClampExtend());
    	}
    }
    
    private class NewScaleBackwardMacroRight extends StatefulCoordinator {
    	public NewScaleBackwardMacroRight() {
    		super(NewScaleBackwardMacroRight.class);
    		addStates(new IntakeMacro());
    		//addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
            addState("Backward 36 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(36+1)), 0));
            //addState("Set Arm Persistent", new ArmSetPersistent(Calibration.ARM_BALANCE_TARGET));
            //addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
            addState("Rotate 35 left", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -35)));
            addState("Backward 9 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(9)), 0));
            addState("Set Arm High Persistent", new ArmSetPersistent(Calibration.ARM_HIGH_TARGET));
            addState("Sleep 1.3 seconds", new SleepCoordinator(1.3));
            addState("Eject cube", new IntakeMove(-0.5,0.5));
            addState("Retract arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
            addState("Retract elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
            addState("Unclamp", new ClampExtend());
    	}
    }
    
    private class NewScaleOppositeMacroLeft extends StatefulCoordinator {
        public NewScaleOppositeMacroLeft() {
            super(NewScaleOppositeMacroLeft.class);
            //addStates(new IntakeMacro());
            //addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
            //addState("Backward 185 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(185+1)), 0));
            //addState("Set Arm Persistent", new ArmSetPersistent(Calibration.ARM_BALANCE_TARGET));
            //addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
            //addState("Backward 50 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(50+1)), 0));
            addState("Rotate 90 right", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 90)));
            //238.5
            addState("Backward 190 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(190+1)), 0));
            addState("Rotate 90 left", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -90)));
            addState("Backward 33 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(33+1)), 0));
            addState("Set Arm High Persistent", new ArmSetPersistent(Calibration.ARM_HIGH_TARGET));
            addState("Sleep 1.3 seconds", new SleepCoordinator(1.3));
            addState("Eject cube", new IntakeMove(-0.5,0.5));
            addState("Retract arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
            addState("Retract elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
            addState("Unclamp", new ClampExtend());
        }
    }
    
    private class NewScaleOppositeMacroRight extends StatefulCoordinator {
        public NewScaleOppositeMacroRight() {
            super(NewScaleOppositeMacroRight.class);
          //addStates(new IntakeMacro());
            //addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
            //addState("Backward 185 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(185+1)), 0));
            //addState("Set Arm Persistent", new ArmSetPersistent(Calibration.ARM_BALANCE_TARGET));
            //addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
            //addState("Backward 50 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(50+1)), 0));
            addState("Rotate 90 left", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -90)));
            addState("Backward 190 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(190+1)), 0));
            addState("Rotate 90 right", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 90)));
            addState("Backward 33 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(33+1)), 0));
            addState("Set Arm High Persistent", new ArmSetPersistent(Calibration.ARM_HIGH_TARGET));
            addState("Sleep 1.3 seconds", new SleepCoordinator(1.3));
            addState("Eject cube", new IntakeMove(-0.7,0.5));
            addState("Retract arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
            addState("Retract elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
            addState("Unclamp", new ClampExtend());
        }
    }
    
    private class DemoStateMacro extends StatefulCoordinator {
        public DemoStateMacro() {
            super(DemoStateMacro.class);
            addState("Forward 18 ft", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 12*18+1), 0));
            addState("Rotate 90 right", new DriveArcadeServoMacro(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 90)));
            addState("Forward 6 ft", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 12*6), 0));
        }
    }
    
    /* Utilities */
    private class DriveArcadeServoMacro extends Coordinator {
        private final double moveSetpoint;
        private final double rotSetpoint;

        private Drive.ArcadeServo arcadeServo;

        public DriveArcadeServoMacro (double moveSetpoint, double rotSetpoint) {
            this.moveSetpoint = moveSetpoint;
            this.rotSetpoint = rotSetpoint;
        }

        @Override
        protected void begin () {
            arcadeServo = robot.drive.new ArcadeServo(moveSetpoint, rotSetpoint);
        }

        @Override
        protected boolean run () {
            arcadeServo.activate();
            return arcadeServo.onTarget.get();
        }
    }

    private class IntakeMacro extends StatefulCoordinator {
        public IntakeMacro() {
            super(IntakeMacro.class);
            addState("Intake cube", new IntakeMove(0.5,0.25));
            //addState("Sleep 0.25 seconds", new SleepCoordinator(0.25));
            addState("Clamp cube", new ClampRetract());
        }
    }
    
    private class SwitchEjectMacro extends StatefulCoordinator {
        public SwitchEjectMacro() {
            super(SwitchEjectMacro.class);
            addState("Eject cube", new IntakeMove(-0.3,1.5));
            // Move back to avoid arm hitting switch fence
            addState("Move back", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -12), 0));
            addState("Move arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
            addState("Wait", new SleepCoordinator(0.2));
            addState("Move elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
            addState("Unclamp", new ClampExtend());
        }
    }
    
    private class LowerMacro extends StatefulCoordinator {
        public LowerMacro() {
            super(LowerMacro.class);
            addState("Back away 24 inches", new DriveArcadeServoMacro(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -24), 0));
            addState("Retract arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
            addState("Retract elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
            addState("Unclamp", new ClampExtend());
        }
    }
}