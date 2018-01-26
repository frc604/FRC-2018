package com._604robotics.robot2018.modes;

import com._604robotics.robot2018.Robot2018;
import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.modules.Drive;
import com._604robotics.robot2018.modules.Elevator;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.flow.Toggle;
import com._604robotics.robotnik.prefabs.inputcontroller.xbox.XboxController;

public class TeleopMode extends Coordinator {

    private final XboxController driver = new XboxController(0);
    private final XboxController manip = new XboxController(1);

    private final Robot2018 robot;

    private final DriveManager driveManager;
    private final ElevatorManager elevatorManager;



    public TeleopMode (Robot2018 robot) {
        driver.leftStick.x.setDeadband(Calibration.TELEOP_DEADBAND);
        driver.leftStick.y.setDeadband(Calibration.TELEOP_DEADBAND);

        driver.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
        driver.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

        driver.rightStick.x.setDeadband(Calibration.TELEOP_DEADBAND);
        driver.rightStick.y.setDeadband(Calibration.TELEOP_DEADBAND);

        driver.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
        driver.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);

        manip.leftStick.x.setDeadband(Calibration.TELEOP_DEADBAND);
        manip.leftStick.y.setDeadband(Calibration.TELEOP_DEADBAND);

        manip.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
        manip.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

        manip.rightStick.x.setDeadband(Calibration.TELEOP_DEADBAND);
        manip.rightStick.y.setDeadband(Calibration.TELEOP_DEADBAND);

        manip.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
        manip.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);

        this.robot = robot;

        driveManager = new DriveManager();
        elevatorManager = new ElevatorManager();
    }

    @Override
    public boolean run () {
        driveManager.run();
        elevatorManager.run();
        return true;
    }
    
    private class ElevatorManager {
    	private final Elevator.Move move;
    	private final Elevator.Hold hold;
    	private final Elevator.Setpoint setpoint;
    	private int i=0;
    	
    	public ElevatorManager() {
    		move = robot.elevator.new Move();
    		hold = robot.elevator.new Hold();
    		setpoint = robot.elevator.new Setpoint();
    	}
    	
    	public void run() {
    		i++;
    		if (i%100==0)
    			System.out.println("Encoder "+robot.elevator.encoderClicks.get());
    		double leftY = manip.leftStick.y.get();
    		boolean buttonY = manip.buttons.y.get();
    		setpoint.target_clicks.set(Calibration.ELEVATOR_Y_TARGET);
    		if( buttonY ) {
    			setpoint.activate();
    		} else {
	    		if( leftY == 0 && manip.buttons.start.get()) {
	    			hold.activate();
	    		} else {
	    			move.liftPower.set(leftY);
	    			move.activate();
	    		}
    		}
    	}
    }
    
    private enum CurrentDrive {
        IDLE, ARCADE, TANK
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
            // Set gears
            gearState.update(driver.buttons.lb.get());
            // Will probably be double solenoid but waiting
            /*if (gearState.isInOnState()) {
                robot.shifter.highGear.activate();
            } else if (gearState.isInOffState()) {
                robot.shifter.lowGear.activate();
            }*/
            // Get Xbox data
            double leftY=driver.leftStick.y.get();
            double rightX=driver.rightStick.x.get();
            double rightY=driver.rightStick.y.get();
            // Flip values if xbox inverted
            inverted.update(driver.buttons.rb.get());
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
}
