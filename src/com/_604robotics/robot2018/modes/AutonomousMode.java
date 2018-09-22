package com._604robotics.robot2018.modes;

import java.io.IOException;
import java.util.TimerTask;

import com._604robotics.marionette.InputRecording;
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
import com._604robotics.robotnik.prefabs.coordinators.SwitchCoordinator;
import com._604robotics.robotnik.prefabs.flow.Pulse;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import com._604robotics.robotnik.utils.AutonMovement;
import com._604robotics.robotnik.utils.PathFinderUtil;
import com._604robotics.robotnik.utils.annotations.Unreal;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class AutonomousMode extends Coordinator {
	private static final Logger logger = new Logger(AutonomousMode.class);

	private final Robot2018 robot;

	private final Coordinator rotateLeftStateMacro;
	private final Coordinator rotateRightStateMacro;
	private final Coordinator forwardSwitchMacro;

	private Coordinator selectedModeMacro;

	public String primaryFileName;
	public String secondaryFileName;

	public static enum PathFollowSide {
		LEFT,
		RIGHT
	}

	public AutonomousMode (Robot2018 robot) {
		this.robot = robot;

		rotateLeftStateMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_STILL_TARGET,
				Calibration.DRIVE_ROTATE_LEFT_TARGET);
		rotateRightStateMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_STILL_TARGET,
				Calibration.DRIVE_ROTATE_RIGHT_TARGET);
		forwardSwitchMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_FORWARD_SWITCH_INCHES, 0);
	}

	@Override
	public void begin () {
		// Filename is prefixed in MarionetteDriver
		primaryFileName = robot.dashboard.marionetteFile.get();
		secondaryFileName = "";

		Coordinator marionetteDriver;

		switch( robot.dashboard.marionetteOutput.get() ) {
		case MANUAL:
			marionetteDriver = new MarionetteDriver(primaryFileName);
			break;
		case SWITCH:
			primaryFileName = Calibration.SWITCH_LEFT_FILENAME;
			secondaryFileName = Calibration.SWITCH_RIGHT_FILENAME;
			marionetteDriver = new MarionetteSwitch();
			break;
		case SCALE_LEFT:
			primaryFileName = Calibration.SCALE_LEFT_FILENAME;
			marionetteDriver = new MarionetteLeftScale();
			break;
		case SCALE_RIGHT:
			primaryFileName = Calibration.SCALE_RIGHT_FILENAME;
			marionetteDriver = new MarionetteRightScale();
			break;
		case CUSTOM_SWITCH:
			primaryFileName = Calibration.CUSTOM_PRIMARY;
			secondaryFileName = Calibration.CUSTOM_SECONDARY;
			marionetteDriver = new CustomMarionetteSwitch();
			break;
		case MANUAL_SWITCH:
			primaryFileName = robot.dashboard.manualPrimaryReadFile.get();
			secondaryFileName = robot.dashboard.manualSecondaryReadFile.get();
			marionetteDriver = new CustomMarionetteSwitch();
			break;
		default:
			marionetteDriver = new FallForwardMacro();
			break;
		}
		// reset arm encoder (again)
		robot.arm.encoder.zero(Calibration.ARM_BOTTOM_LOCATION);

		switch (robot.dashboard.autonMode.get()) {
		case LEFT_DELAYED_BASELINE_CROSS:
			selectedModeMacro = new LeftDelayedCrossMacro();
			break;
		case RIGHT_DELAYED_BASELINE_CROSS:
			selectedModeMacro = new RightDelayedCrossMacro();
			break;
		case CENTER_SWITCH:
			selectedModeMacro = new CenterSwitchMacro();
			break;
		case BACKWARD_CENTER_SWITCH:
			selectedModeMacro = new BackwardsCenterSwitchMacro();
			break;
		case LEFT_SWITCH:
			selectedModeMacro = new LeftSwitchMacro();
			break;
		case RIGHT_SWITCH:
			selectedModeMacro = new RightSwitchMacro();
			break;
		case LEFT_SCALE_WITH_CROSS:
			selectedModeMacro = new LeftScaleMacro();
			break;
		case RIGHT_SCALE_WITH_CROSS:
			selectedModeMacro = new RightScaleMacro();
			break;
		case LEFT_SCALE_WITHOUT_CROSS:
			selectedModeMacro = new LeftScaleSameOnlyMacro();
			break;
		case RIGHT_SCALE_WITHOUT_CROSS:
			selectedModeMacro = new RightScaleSameOnlyMacro();
			break;
		case LEFT_SCALE_HALF_CROSS:
			selectedModeMacro = new LeftScaleHalfCrossMacro();
			break;
		case RIGHT_SCALE_HALF_CROSS:
			selectedModeMacro = new RightScaleHalfCrossMacro();
			break;
		case ROTATE_LEFT_TEST:
			selectedModeMacro = rotateLeftStateMacro;
			break;
		case ROTATE_RIGHT_TEST:
			selectedModeMacro = rotateRightStateMacro;
			break;
		case FAILSAFE_FORWARD_12:
			selectedModeMacro = new FallForwardMacro();
			break;
		case FAILSAFE_BACKWARD_12:
			selectedModeMacro = new FallBackMacro();
			break;
		case DEMO_NEW_AUTON:
			selectedModeMacro = new DemoStateMacro();
			break;
		case FORWARD_SWITCH:
			selectedModeMacro = forwardSwitchMacro;
			break;
			//            case CENTER_SWITCH_LEFT:
			//                selectedModeMacro = new CenterMacroLeft();
			//                break;
			//            case CENTER_SWITCH_RIGHT:
			//                selectedModeMacro = new CenterMacroRight();
			//                break;
			//            case SWERVE_SCALE_OPPOSITE_LEFT:
			//                selectedModeMacro = new NewScaleOppositeMacroLeft();
			//                break;
			//            case SWITCH_FORWARD:
			//                selectedModeMacro = new SwitchForwardBackupMacro();
			//                break;
		case NEW_SCALE_BACKWARD:
			selectedModeMacro = new NewScaleBackwardMacroLeft();
			break;
			//            case BALANCED_LEFT_TURN_TEST:
			//            	selectedModeMacro = new BalancedLeftTurnMacro();
			//            	break;
			//            case SWEPT_LEFT_TURN_TEST:
			//            	selectedModeMacro = new SweptLeftTurnMacro();
			//            	break;
			//            case BALANCED_SWEPT_LEFT_TURN_TEST:
			//            	selectedModeMacro = new BalancedSweptLeftTurnMacro();
			//            	break;
			//            case BALANCED_RIGHT_TURN_TEST:
			//            	selectedModeMacro = new BalancedRightTurnMacro();
			//            	break;
			//            case SWEPT_RIGHT_TURN_TEST:
			//            	selectedModeMacro = new SweptRightTurnMacro();
			//            	break;
			//            case BALANCED_SWEPT_RIGHT_TURN_TEST:
			//            	selectedModeMacro = new BalancedSweptRightTurnMacro();
			//            	break;
		case MARIONETTE:
			selectedModeMacro = marionetteDriver;
			break;
		case OFF:
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

	private class CustomMarionetteSwitch extends SwitchCoordinator {
		public CustomMarionetteSwitch() {
			super(CustomMarionetteSwitch.class);
			addDefault(new SleepCoordinator(0.1)); // Lucky randomness guaranteed by coin flip
			addCase(new String[]{"LLL", "LLR", "LRL", "LRR"}, new CustomMarionetteDriver(primaryFileName));
			addCase(new String[]{"RLL", "RLR", "RRL", "RRR"}, new CustomMarionetteDriver(secondaryFileName));
		}
	}

	private class MarionetteSwitch extends SwitchCoordinator {
		public MarionetteSwitch() {
			super(MarionetteSwitch.class);
			addDefault(new SleepCoordinator(0.1)); // Lucky randomness guaranteed by coin flip
			addCase(new String[]{"LLL", "LLR", "LRL", "LRR"}, new MarionetteDriver(primaryFileName));
			addCase(new String[]{"RLL", "RLR", "RRL", "RRR"}, new MarionetteDriver(secondaryFileName));
		}
	}

	private class MarionetteLeftScale extends SwitchCoordinator {
		public MarionetteLeftScale() {
			super(MarionetteLeftScale.class);
			addDefault(new SleepCoordinator(0.1)); // Lucky randomness guaranteed by coin flip
			addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new MarionetteDriver(primaryFileName));
			addCase(new String[]{"LRL", "LRR", "RRL", "RRR"}, new LeftScaleMacro());
		}
	}

	private class MarionetteRightScale extends SwitchCoordinator {
		public MarionetteRightScale() {
			super(MarionetteRightScale.class);
			addDefault(new SleepCoordinator(0.1)); // Lucky randomness guaranteed by coin flip
			addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new RightScaleMacro());
			addCase(new String[]{"LRL", "LRR", "RRL", "RRR"}, new MarionetteDriver(primaryFileName));
		}
	}

	private class CustomMarionetteDriver extends Coordinator {
		private String fileName;

		public CustomMarionetteDriver(String fileName) {
			this.fileName = fileName;
		}

		@Override
		protected void begin () {
			logger.info("Loading Marionette recording from \"" + fileName + "\"");
			final InputRecording recording;
			try {
				recording = InputRecording.load("/home/lvuser/" + fileName);
			} catch (IOException e) {
				throw new RuntimeException(e);
			}

			logger.info("Starting Marionette playback");
			robot.teleopMode.startPlayback(recording);
			robot.teleopMode.start();
		}

		@Override
		protected boolean run () {
			return robot.teleopMode.execute();
		}

		@Override
		protected void end () {
			logger.info("Stopping Marionette playback");
			robot.teleopMode.stop();
			robot.teleopMode.stopPlayback();
		}
	}

	private class MarionetteDriver extends Coordinator {
		private String fileName;

		public MarionetteDriver(String fileName) {
			this.fileName = robot.dashboard.filePrefix.get() + fileName;
		}

		@Override
		protected void begin () {
			logger.info("Loading Marionette recording from \"" + robot.dashboard.filePrefix.get() + fileName + "\"");
			final InputRecording recording;
			try {
				recording = InputRecording.load("/home/lvuser/" + robot.dashboard.filePrefix.get() + fileName);
			} catch (IOException e) {
				throw new RuntimeException(e);
			}

			logger.info("Starting Marionette playback");
			robot.teleopMode.startPlayback(recording);
			robot.teleopMode.start();
		}

		@Override
		protected boolean run () {
			return robot.teleopMode.execute();
		}

		@Override
		protected void end () {
			logger.info("Stopping Marionette playback");
			robot.teleopMode.stop();
			robot.teleopMode.stopPlayback();
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

	private class LeftDelayedCrossMacro extends StatefulCoordinator {
		public LeftDelayedCrossMacro() {
			super(LeftDelayedCrossMacro.class);
			addStates(new IntakeMacro());
			addState("Wait for 7 seconds", new SleepCoordinator(7));
			addStates(new LeftSwitchMacro());
		}
	}

	private class RightDelayedCrossMacro extends StatefulCoordinator {
		public RightDelayedCrossMacro() {
			super(RightDelayedCrossMacro.class);
			addStates(new IntakeMacro());
			addState("Wait for 7 seconds", new SleepCoordinator(7));
			addStates(new RightSwitchMacro());
		}
	}

	private class FallBackMacro extends StatefulCoordinator {
		public FallBackMacro() {
			super(FallBackMacro.class);
			addStates(new IntakeMacro());
			//
			addState("Pathfind back 144in", new PathStraight( PathFinderUtil.inchesToMeters( 144 ), true ));
		}
	}

	private class FallForwardMacro extends StatefulCoordinator {
		public FallForwardMacro() {
			super(FallForwardMacro.class);
			addStates(new IntakeMacro());
		}
	}

	private class LeftSwitchMacro extends StatefulCoordinator {
		public LeftSwitchMacro() {
			super(LeftSwitchMacro.class);
			addStates(new IntakeMacro());
			addState("Raise elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_RAISE_TARGET));
			addState("Wait for elevator", new SleepCoordinator(0.3));
			addState("Raise arm", new ArmSetPersistent(Calibration.ARM_MID_TARGET));
			/*addState("Pathfind forward 144in, 90 right, forward 24in", new PathFollower( new Waypoint[] {
					new Waypoint( 0, 0, 0 ),
					new Waypoint( PathFinderUtil.feetToMeters( 9 ), 0, 0 ),
					new Waypoint( PathFinderUtil.feetToMeters( 12 ), PathFinderUtil.feetToMeters( -2 ), Pathfinder.d2r(-90) ),

			} ));*/
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
			/*addState("Pathfind forward 144in, 90 left, forward 24in", new PathFollower( new Waypoint[] {
					new Waypoint( 0, 0, 0 ),
					new Waypoint( PathFinderUtil.feetToMeters( 9 ), 0, 0 ),
					new Waypoint( PathFinderUtil.feetToMeters( 12 ), PathFinderUtil.feetToMeters( 2 ), Pathfinder.d2r(90) ),

			} ));*/
			addState("Switch decision", new RightSideSwitchDecisionMacro());
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
			// System.out.println("Move error is " + getMoveError() + ", Rot error is " + getRotError());
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

	private class CenterSwitchMacro extends StatefulCoordinator {
		public CenterSwitchMacro() {
			super(CenterSwitchMacro.class);
			addStates(new IntakeMacro());
			addState("Raise elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_SWITCH_CLEAR));
			addState("Wait for elevator", new SleepCoordinator(0.3));
			addState("Raise arm", new ArmSetPersistent(Calibration.ARM_MID_TARGET));
			// Choose based on FMS Game Data
			addState("Switch choosing", new CenterSwitchChooserMacro());
			//addState("Pathfind forward 23in", new PathStraight( PathFinderUtil.inchesToMeters( 23 ) ) );
			addStates(new SwitchEjectMacro());
		}
	}

	// Comment me out
	@Unreal("Experimenting with a backwards jerk to deploy outer intake")
	private class BackwardsCenterSwitchMacro extends StatefulCoordinator {
		public BackwardsCenterSwitchMacro() {
			super(BackwardsCenterSwitchMacro.class);
			addState("Jerk backwards", new TimedMacro(-1, 0, 0.5));
			addState("Rotate 180", new ArcadePIDCoordinator(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 180)));
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
			//addState("Pathfind back 219in", new PathStraight( PathFinderUtil.inchesToMeters(219), true ) );
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
			//addState("Pathfind back 219in", new PathStraight( PathFinderUtil.inchesToMeters(219), true ) );
			//addState("Set Arm Persistent", new ArmSetPersistent(Calibration.ARM_BALANCE_TARGET));
			//addState("Sleep 0.2 seconds", new SleepCoordinator(0.2));
			addState("Scale chooser", new LeftScaleChooserSameOnlyMacro());
		}
	}

	private class LeftScaleHalfCrossMacro extends StatefulCoordinator {
		public LeftScaleHalfCrossMacro() {
			super(LeftScaleHalfCrossMacro.class);
			addStates(new IntakeMacro());
			addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
			//addState("Pathfind back 219in", new PathStraight( PathFinderUtil.inchesToMeters(219), true ) );
			addState("Scale chooser", new LeftScaleChooserHalfCrossMacro());
		}
	}

	private class RightScaleMacro extends StatefulCoordinator {
		public RightScaleMacro() {
			super(RightScaleMacro.class);
			addStates(new IntakeMacro());
			addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
			//addState("Pathfind back 219in", new PathStraight( PathFinderUtil.inchesToMeters(219), true ) );
			//addState("Set Arm Persistent", new ArmSetPersistent(Calibration.ARM_BALANCE_TARGET));
			//addState("Sleep 0.2 seconds", new SleepCoordinator(0.2));
			addState("Scale chooser", new RightScaleChooserMacro());
		}
	}

	private class RightScaleHalfCrossMacro extends StatefulCoordinator {
		public RightScaleHalfCrossMacro() {
			super(RightScaleHalfCrossMacro.class);
			addStates(new IntakeMacro());
			addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
			//addState("Pathfind back 219in", new PathStraight( PathFinderUtil.inchesToMeters(219), true ) );
			addState("Scale chooser", new RightScaleChooserHalfCrossMacro());
		}
	}

	private class RightScaleSameOnlyMacro extends StatefulCoordinator {
		public RightScaleSameOnlyMacro() {
			super(RightScaleSameOnlyMacro.class);
			addStates(new IntakeMacro());
			addState("Set Elevator Persistent", new ElevatorSetPersistent(Calibration.ELEVATOR_MID_TARGET));
			//addState("Pathfind back 219in", new PathStraight( PathFinderUtil.inchesToMeters(219), true ) );
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

	private class LeftScaleChooserHalfCrossMacro extends SwitchCoordinator {
		public LeftScaleChooserHalfCrossMacro() {
			super(LeftScaleChooserHalfCrossMacro.class);
			addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new NewScaleBackwardMacroLeft());
			addCase(new String[]{"LRL", "LRR", "RRL", "RRR"}, new NewScaleHalfCrossMacroLeft());
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

	private class RightScaleChooserHalfCrossMacro extends SwitchCoordinator {
		public RightScaleChooserHalfCrossMacro() {
			super(RightScaleChooserHalfCrossMacro.class);
			addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new NewScaleHalfCrossMacroRight());
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
			/*addState("Pathfind forward 82.74in, up 73.74in", new PathFollower( new Waypoint[] {
					new Waypoint( 0, 0, 0 ),
					new Waypoint( PathFinderUtil.inchesToMeters(82.7401153), PathFinderUtil.inchesToMeters(-73.7401153), 0 )
			} ));*/
		}
	}

	private class CenterMacroRight extends StatefulCoordinator {
		public CenterMacroRight() {
			super(CenterMacroRight.class);
			/*addState("Pathfind forward 76.74in, up 53.74in", new PathFollower( new Waypoint[] {
					new Waypoint( 0, 0, 0 ),
					new Waypoint( PathFinderUtil.inchesToMeters(76.7401153), PathFinderUtil.inchesToMeters(53.7401153), 0 ),
			} ));*/
		}
	}

	private class NewScaleBackwardMacroLeft extends StatefulCoordinator {
		public NewScaleBackwardMacroLeft() {
			super(NewScaleBackwardMacroLeft.class);
			addStates(new IntakeMacro());
			/*addState("Pathfind back 43.91491226573395in, down 6in, angle -35deg", new PathFollower( new Waypoint[] {
				new Waypoint( 0, 0, 0 ),
				new Waypoint( PathFinderUtil.inchesToMeters(16), 0, 0 ),
				new Waypoint( PathFinderUtil.inchesToMeters(43.91491226573395), PathFinderUtil.inchesToMeters(-6), Pathfinder.d2r(-35) )
			}, true ));*/
			addState("Set Arm High Persistent", new ArmSetPersistent(Calibration.ARM_HIGH_TARGET));
			addState("Sleep 1.3 seconds", new SleepCoordinator(1.3));
			addState("Eject cube", new IntakeMove(-0.4,0.5));
			addState("Retract arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
			addState("Retract elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
			addState("Unclamp", new ClampExtend());
		}
	}

	private class NewScaleBackwardMacroRight extends StatefulCoordinator {
		public NewScaleBackwardMacroRight() {
			super(NewScaleBackwardMacroRight.class);
			addStates(new IntakeMacro());
			/*addState("Pathfind back 43.91491226573395in, up 6in, angle 35deg", new PathFollower( new Waypoint[] {
				new Waypoint( 0, 0, 0 ),
				new Waypoint( PathFinderUtil.inchesToMeters(16), 0, 0 ),
				new Waypoint( PathFinderUtil.inchesToMeters(43.91491226573395), PathFinderUtil.inchesToMeters(6), Pathfinder.d2r(35) )
			}, true ));*/
			addState("Set Arm High Persistent", new ArmSetPersistent(Calibration.ARM_HIGH_TARGET));
			addState("Sleep 1.3 seconds", new SleepCoordinator(1.3));
			addState("Eject cube", new IntakeMove(-0.4,0.5));
			addState("Retract arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
			addState("Retract elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
			addState("Unclamp", new ClampExtend());
		}
	}

	private class NewScaleOppositeMacroLeft extends StatefulCoordinator {
		public NewScaleOppositeMacroLeft() {
			super(NewScaleOppositeMacroLeft.class);
			addState("Rotate 86 right", new ArcadePIDCoordinator(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 86)));
			//238.5
            //addState("Pathfind back 193in", new PathStraight( PathFinderUtil.inchesToMeters(193), true ));
            addState("Rotate 90 left", new ArcadePIDCoordinator(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -90)));
            //addState("Pathfind back 27in", new PathStraight( PathFinderUtil.inchesToMeters(27), true ));
            addState("Set Arm High Persistent", new ArmSetPersistent(Calibration.ARM_HIGH_TARGET));
			addState("Sleep 1.3 seconds", new SleepCoordinator(1.3));
			addState("Eject cube", new IntakeMove(-0.4,0.5));
			addState("Retract arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
			addState("Retract elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
			addState("Unclamp", new ClampExtend());
		}
	}

	private class NewScaleOppositeMacroRight extends StatefulCoordinator {
		public NewScaleOppositeMacroRight() {
			super(NewScaleOppositeMacroRight.class);
			// Angles have been left separate due to concerns about hitting the
			// ramp at an angle instead of dead-on.
			addState("Rotate 90 left", new ArcadePIDCoordinator(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -90)));
			//addState("Pathfind back 193in", new PathStraight( PathFinderUtil.inchesToMeters(193), true ) );
			addState("Rotate 90 right", new ArcadePIDCoordinator(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 90)));
			//addState("Pathfind back 27in", new PathStraight( PathFinderUtil.inchesToMeters(27), true ));
			addState("Set Arm High Persistent", new ArmSetPersistent(Calibration.ARM_HIGH_TARGET));
			addState("Sleep 1.3 seconds", new SleepCoordinator(1.3));
			addState("Eject cube", new IntakeMove(-0.4,0.5));
			addState("Retract arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
			addState("Retract elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
			addState("Unclamp", new ClampExtend());
		}
	}

	private class NewScaleHalfCrossMacroLeft extends StatefulCoordinator {
		public NewScaleHalfCrossMacroLeft() {
			super(NewScaleHalfCrossMacroLeft.class);
			addState("Rotate 90 right", new ArcadePIDCoordinator(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 90)));
			//addState("Pathfind back 100in", new PathStraight( PathFinderUtil.inchesToMeters(100), true ) );
		}
	}

	private class NewScaleHalfCrossMacroRight extends StatefulCoordinator {
		public NewScaleHalfCrossMacroRight() {
			super(NewScaleHalfCrossMacroRight.class);
			addState("Rotate 90 left", new ArcadePIDCoordinator(0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -90)));
			addState("Pathfind back 100in", new PathStraight( PathFinderUtil.inchesToMeters(100), true ) );
		}
	}

	private class DemoStateMacro extends StatefulCoordinator {
		public DemoStateMacro() {
			super(DemoStateMacro.class);
			addState("PATHFIND MY FRIEND", new PathFollower( new Waypoint[] {
				new Waypoint( 0, 0, 0 ),
				new Waypoint( PathFinderUtil.feetToMeters(4), PathFinderUtil.feetToMeters(4), Pathfinder.d2r(90) )
			}, true ));
		}
	}

	/* Utilities */
	private class IntakeMacro extends StatefulCoordinator {
		public IntakeMacro() {
			super(IntakeMacro.class);
			addState("Intake cube", new IntakeMove(0.5,0.25));
			addState("Clamp cube", new ClampRetract());
		}
	}

	private class SwitchEjectMacro extends StatefulCoordinator {
		public SwitchEjectMacro() {
			super(SwitchEjectMacro.class);
			addState("Eject cube", new IntakeMove(-0.3,1.5));
			// Move back to avoid arm hitting switch
			addState("Pathfind back 1ft", new PathStraight( PathFinderUtil.feetToMeters(1), true ));
			addState("Move arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
			addState("Wait", new SleepCoordinator(0.2));
			addState("Move elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
			addState("Unclamp", new ClampExtend());
		}
	}

	private class LowerMacro extends StatefulCoordinator {
		public LowerMacro() {
			super(LowerMacro.class);
			addState("Pathfind back 24in", new PathStraight( PathFinderUtil.inchesToMeters( 24 ), true ));
			addState("Retract arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
			addState("Retract elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
			addState("Unclamp", new ClampExtend());
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

	// Backup in case switch game data FMS malfunctions
	private class SwitchForwardBackupMacro extends StatefulCoordinator {
		public SwitchForwardBackupMacro() {
			super(SwitchForwardBackupMacro.class);
			addStates(new IntakeMacro());
			addState("Pathfind forward 120in", new PathStraight( PathFinderUtil.inchesToMeters( 120 ) ));
			addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
			addStates(new SwitchEjectMacro());
			//addState("Rotate 180 left", new ArcadePIDCoordinator(0,AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -180)));
			//addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
			//addState("Forward 12 feet", new ArcadePIDCoordinator(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 12*12+1), 0));
		}
	}

	private class PathFollower extends Coordinator {
		private Trajectory path;
		private EncoderFollower leftFollower;
		private EncoderFollower rightFollower;
		private java.util.Timer timer;
		private SmartTimer timeElapsed;
		private Drive.TankDrive tankDrive;
		private Pulse PIDTargetPulse=new Pulse();

		public PathFollower( Waypoint[] waypoints ) {
			this( Pathfinder.generate( waypoints, Calibration.PATHFINDER_CONFIG ), false );
		}

		public PathFollower( Waypoint[] waypoints, boolean reversePath ) {
			this( Pathfinder.generate( waypoints, Calibration.PATHFINDER_CONFIG ), reversePath );
		}

		public PathFollower( Trajectory path ) {
			this( path, false );
		}

		public PathFollower( Trajectory path, boolean reverseDrive ) {
			this.path = path;
			timeElapsed = new SmartTimer();
			tankDrive = robot.drive.new TankDrive( false );
			generateTankPath( reverseDrive );
		}

		private void generateTankPath( boolean reverseDrive ) {
			TankModifier modifier = new TankModifier(path).modify( Calibration.Pathfinder.ROBOT_WIDTH, Calibration.PATHFINDER_CONFIG );

			if( reverseDrive ) {
				leftFollower = new EncoderFollower( Pathfinder.reverseTrajectory( modifier.getLeftTrajectory( ) ) );
				rightFollower = new EncoderFollower( Pathfinder.reverseTrajectory( modifier.getRightTrajectory() ) );
			} else {
				leftFollower = new EncoderFollower( modifier.getLeftTrajectory() );
				rightFollower = new EncoderFollower( modifier.getRightTrajectory() );
			}
		}

		class PathFollowTask extends TimerTask {
			private double prevAngleError=0;
			private PathFollowSide side;
			private double next_dt = 0;

			private double clamp(double value, double low, double high) {
				return Math.max(low, Math.min(value, high));
			}

			PathFollowTask (PathFollowSide side) {
				this.side=side;
			}

			@Override
			public void run() {
				tankDrive.activate();
				int encoderPos;
				double rawPow;
				Trajectory.Segment prev_seg;
				Trajectory.Segment seg;
				if (side==PathFollowSide.LEFT) {
					encoderPos = robot.drive.leftClicks.get();
					rawPow = leftFollower.calculate(encoderPos);
					prev_seg = leftFollower.prevSegment();
					seg = leftFollower.getSegment();
				} else {//if side==PathFollowSide.RIGHT
					encoderPos=robot.drive.rightClicks.get();
					rawPow = rightFollower.calculate(encoderPos);
					prev_seg = rightFollower.prevSegment();
					seg = rightFollower.getSegment();
				}

				next_dt = seg.dt;

				// Calculated curvature scales with velocity
				// Keeping old implied scaling since faster movement implies more curvature correction
				// Use harmonic mean because curvature is 1/radius
				double scaleVel=2*leftFollower.getSegment().velocity*rightFollower.getSegment().velocity;
				if (scaleVel!=0) {
					scaleVel/=(leftFollower.getSegment().velocity+rightFollower.getSegment().velocity);
				}
				double curvature = PathFinderUtil.getScaledCurvature(prev_seg,seg,scaleVel);

				double normcurv=PathFinderUtil.getNormalizedCurvature(prev_seg, seg);
				//System.out.println("Normcurv is "+normcurv+" for "+side.toString());

				// Raw heading stuff here due to side selections
				double degreeHeading = AutonMovement.clicksToDegrees(Calibration.DRIVE_PROPERTIES,
						robot.drive.leftClicks.get()-robot.drive.rightClicks.get());
				//System.out.print("Current heading is "+degreeHeading);

				// Both headings are the same
				double desiredHeading = leftFollower.getHeading();
				desiredHeading=Pathfinder.r2d(desiredHeading);
				desiredHeading=Pathfinder.boundHalfDegrees(desiredHeading);
				desiredHeading*=-1;
				// Pathfinder heading is counterclockwise math convention
				// We are using positive=right clockwise convention

				double angleError = desiredHeading-degreeHeading;
				System.out.println("Angle error is "+angleError);

				// Convert back into radians for consistency
				angleError = Pathfinder.d2r(angleError);
				double dAngleError=angleError-prevAngleError;
				dAngleError/=seg.dt;

				double kappa_val=Calibration.Pathfinder.K_KAPPA*curvature;
				double pTheta_val=Calibration.Pathfinder.K_PTHETA_0/(Calibration.Pathfinder.K_PTHETA_DECAY*normcurv*normcurv+1);
				pTheta_val*=angleError;
				double dTheta_val=Calibration.Pathfinder.K_DTHETA_0/(Calibration.Pathfinder.K_DTHETA_DECAY*normcurv*normcurv+1);
				dTheta_val*=dAngleError;

				dTheta_val=clamp(dTheta_val,-pTheta_val,pTheta_val);

				/*
				double deshed=-Pathfinder.boundHalfRadians(leftFollower.getHeading());
				System.out.println("Equal "+(deshed-Pathfinder.d2r(desiredHeading)));
				*/

				if (side==PathFollowSide.LEFT) {
					tankDrive.leftPower.set(rawPow+kappa_val
							+pTheta_val+dTheta_val);
				} else { // RIGHT side
					tankDrive.rightPower.set(rawPow-kappa_val
							-pTheta_val-dTheta_val);
				}
				prevAngleError=angleError;

				if( side==PathFollowSide.LEFT ) {
					timer.schedule( new PathFollowTask( PathFollowSide.LEFT), (long) ( 1000*getNextdt() ) );
				} else if( side == PathFollowSide.RIGHT ) {
					timer.schedule( new PathFollowTask( PathFollowSide.RIGHT), (long) (1000*getNextdt()) );
				}
			}

			double getNextdt() {
				return next_dt;
			}
		}


		@Override
		public void begin() {
			System.out.println("ERROR: Begin");
			timer = new java.util.Timer();
			robot.drive.resetSensors();
			System.out.println("ERROR: left is "+robot.drive.leftClicks.get());
			leftFollower.configurePIDVA(Calibration.Pathfinder.KP, Calibration.Pathfinder.KI, Calibration.Pathfinder.KD,
					Calibration.Pathfinder.KV, Calibration.Pathfinder.KA);
			rightFollower.configurePIDVA(Calibration.Pathfinder.KP, Calibration.Pathfinder.KI, Calibration.Pathfinder.KD,
					Calibration.Pathfinder.KV, Calibration.Pathfinder.KA);
			leftFollower.configureEncoder(0, 250, 0.12732); // 5 in diameter
			rightFollower.configureEncoder(0, 250, 0.12732);
			leftFollower.reset();
			rightFollower.reset();
			timer.schedule(new PathFollowTask(PathFollowSide.LEFT), 0);
			timer.schedule(new PathFollowTask(PathFollowSide.RIGHT), 0);
			tankDrive.activate();
			timeElapsed.start();
		}

		@Override
		public boolean run() {
			tankDrive.activate();
			return timeElapsed.runUntil(0.6, new Runnable() {
				@Override
				public void run() {
					boolean targetReached = (leftFollower.isFinished() && rightFollower.isFinished());
					if (!targetReached) {
						timeElapsed.reset();
						PIDTargetPulse.update(true);
					} else {
						PIDTargetPulse.update(false);
					}
					if (PIDTargetPulse.isFallingEdge()) {
						System.out.println("========Finished========");
					}
				}
			});
		}

		@Override
		public void end() {
			timer.cancel();
			System.out.println("ERROR: end");
			leftFollower.reset();
			rightFollower.reset();
			timeElapsed.stopAndReset();
		}
	}

	private class PathStraight extends PathFollower {
		public PathStraight( double meters ) {
			this( meters, false );
		}

		public PathStraight( double meters, boolean reverseDrive ) {
			super( Pathfinder.generate( new Waypoint[]{
					new Waypoint( 0,0,0 ),
					new Waypoint( meters, 0, 0 ),
			}, Calibration.PATHFINDER_CONFIG ), reverseDrive );
		}
	}


}
