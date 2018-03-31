package com._604robotics.robot2018.modes;

import com._604robotics.robot2018.Robot2018;
import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.modules.*;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.coordinators.*;
import com._604robotics.robotnik.utils.AutonMovement;

import java.util.EnumMap;

public class AutonomousMode extends StatefulCoordinator {
    private final Robot2018 robot;
    private final EnumMap<Dashboard.AutonMode, Coordinator> autonModes = new EnumMap<>(Dashboard.AutonMode.class);

    public AutonomousMode (Robot2018 robot) {
        super(AutonomousMode.class);

        this.robot = robot;

        autonModes.put(Dashboard.AutonMode.CENTER_SWITCH, new CenterSwitchMacro());

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

        autonModes.put(Dashboard.AutonMode.NEW_SCALE_BACKWARD_LEFT, new NewScaleBackwardLeftMacro());
        autonModes.put(Dashboard.AutonMode.NEW_SCALE_BACKWARD_RIGHT, new NewScaleBackwardRightMacro());

        addState("Zero arm encoder", new ActionCoordinator(robot.arm.zero, () -> true));

        addState("Execute selected autonomous mode", new Coordinator() {
            private Coordinator selectedMode;

            @Override
            protected void begin () {
                selectedMode = autonModes.getOrDefault(robot.dashboard.autonMode.get(), null);

                if (selectedMode != null) {
                    selectedMode.start();
                }
            }

            @Override
            protected boolean run () {
                if (selectedMode == null) {
                    return false;
                }

                return selectedMode.execute();
            }

            @Override
            protected void end () {
                if (selectedMode != null) {
                    selectedMode.stop();
                }
            }
        });
    }

    private class GrabPreloadedCubeMacro extends StatefulCoordinator {
        public GrabPreloadedCubeMacro () {
            super(GrabPreloadedCubeMacro.class);

            addState("Run intake",
                    new TimeLimitCoordinator(0.25, new ActionCoordinator(robot.intake.new Run(0.5))));
        }
    }

    private class DriveArcadeServoMacro extends Coordinator {
        private final double moveSetpoint;
        private final double rotSetpoint;

        private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(moveSetpoint, rotSetpoint);

        public DriveArcadeServoMacro (double moveSetpoint, double rotSetpoint) {
            this.moveSetpoint = moveSetpoint;
            this.rotSetpoint = rotSetpoint;
        }

        @Override
        protected boolean run () {
            arcadeServo.activate();
            return arcadeServo.onTarget.get();
        }
    }

    private class CenterSwitchMacro extends StatefulCoordinator {
        public CenterSwitchMacro () {
            super(CenterSwitchMacro.class);

            addState("Grab preloaded cube", new GrabPreloadedCubeMacro());

            addState("Raise elevator", new TimeLimitCoordinator(0.3, new ActionCoordinator(robot.elevator.raise)));

            // Forward distance between front bumper and scale -76 XX
            addState("Drive away from wall", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 23),0);

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Turn toward switch based on game data", new CenterSwitchDecisionMacro());

            addState("Drive toward switch", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 23),0);

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Eject cube", new SwitchEjectMacro());
        }
    }

    private class LeftSwitchMacro extends StatefulCoordinator {
        public LeftSwitchMacro() {
            super(LeftSwitchMacro.class);

            addState("Grab preloaded cube", new GrabPreloadedCubeMacro());

            addState("Raise elevator", new TimeLimitCoordinator(0.3, new ActionCoordinator(robot.elevator.raise)));

            addState("Drive away from wall", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                    AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 144 + 1), 0);

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Turn toward switch", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 90));

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Drive toward switch", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 24), 0);

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Continue based on game data", new LeftSideSwitchDecisionMacro());
        }
    }

    private class RightSwitchMacro extends StatefulCoordinator {
        public RightSwitchMacro() {
            super(RightSwitchMacro.class);

            addState("Grab preloaded cube", new GrabPreloadedCubeMacro());

            addState("Raise elevator", new TimeLimitCoordinator(0.3, new ActionCoordinator(robot.elevator.raise)));

            addState("Drive away from wall", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 144 + 1), 0);

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Turn toward switch", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -90));

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Drive toward switch", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, (24)), 0);

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Continue based on game data", new RightSideSwitchDecisionMacro());
        }
    }

    private class CenterSwitchDecisionMacro extends SwitchCoordinator {
        public CenterSwitchDecisionMacro () {
            super(CenterSwitchDecisionMacro.class);

            addDefault(new CenterLeftSwitchMacro()); // Lucky randomness guaranteed by coin flip
            addCase(new String[]{"LLL", "LLR", "LRL", "LRR"}, new CenterLeftSwitchMacro());
            addCase(new String[]{"RLL", "RLR", "RRL", "RRR"}, new CenterRightSwitchMacro());
        }
    }

    private class LeftSideSwitchDecisionMacro extends SwitchCoordinator {
        public LeftSideSwitchDecisionMacro() {
            super(LeftSideSwitchDecisionMacro.class);

            addDefault(new CancelSwitchMacro());
            addCase(new String[]{"LLL", "LLR", "LRL", "LRR"}, new SwitchEjectMacro());
            addCase(new String[]{"RLL", "RLR", "RRL", "RRR"}, new CancelSwitchMacro());
        }
    }

    private class RightSideSwitchDecisionMacro extends SwitchCoordinator {
        public RightSideSwitchDecisionMacro() {
            super(RightSideSwitchDecisionMacro.class);
            addDefault(new CancelSwitchMacro());
            addCase(new String[]{"LLL", "LLR", "LRL", "LRR"}, new CancelSwitchMacro());
            addCase(new String[]{"RLL", "RLR", "RRL", "RRR"}, new SwitchEjectMacro());
        }
    }

    private class CenterLeftSwitchMacro extends StatefulCoordinator {
        public CenterLeftSwitchMacro () {
            super(CenterLeftSwitchMacro.class);

            addState("Turn left", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -45));

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Drive toward switch", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 76+15+1),0);

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Turn toward switch", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 45));

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });
        }
    }

    private class CenterRightSwitchMacro extends StatefulCoordinator {
        public CenterRightSwitchMacro () {
            super(CenterLeftSwitchMacro.class);

            addState("Turn right", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 45));

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Drive toward switch", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 76+1),0);

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Turn toward switch", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -45));

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });
        }
    }

    private class SwitchEjectMacro extends StatefulCoordinator {
        public SwitchEjectMacro() {
            super(SwitchEjectMacro.class);

            addState("Eject cube", new TimeLimitCoordinator(1.5, new Coordinator() {
                private final Intake.Run runIntake = robot.intake.new Run(-0.3);

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    runIntake.activate();
                    return true;
                }
            }));

            // Scoot back to avoid arm hitting switch fence
            addState("Back away from switch", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -12), 0);

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Lower arm", new TimeLimitCoordinator(0.2, new Coordinator() {
                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.low.activate();
                    return true;
                }
            }));

            addState("Lower elevator, disengage, and wait", new Coordinator() {
                @Override
                protected boolean run () {
                    robot.elevator.low.activate();
                    robot.arm.low.activate();
                    robot.clamp.release.activate();
                    return true;
                }
            });
        }
    }

    private class CancelSwitchMacro extends StatefulCoordinator {
        public CancelSwitchMacro () {
            super(CancelSwitchMacro.class);

            addState("Back away from switch", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -24), 0);

                @Override
                protected boolean run () {
                    robot.elevator.raise.activate();
                    robot.arm.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Disengage and wait", new Coordinator() {
                @Override
                protected boolean run () {
                    robot.elevator.low.activate();
                    robot.arm.low.activate();
                    robot.clamp.release.activate();
                    return true;
                }
            });
        }
    }

    private class FallBackMacro extends StatefulCoordinator {
        public FallBackMacro() {
            super(FallBackMacro.class);

            addState("Grab preloaded cube", new GrabPreloadedCubeMacro());

            addState("Drive backward", new DriveArcadeServoMacro(
                    AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(144+1)), 0));
        }
    }
    
    private class FallForwardMacro extends StatefulCoordinator {
        public FallForwardMacro() {
            super(FallForwardMacro.class);

            addState("Grab preloaded cube", new GrabPreloadedCubeMacro());

            addState("Drive forward", new DriveArcadeServoMacro(
                    AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, (144+1)), 0));
        }
    }

    private class LeftScaleMacro extends StatefulCoordinator {
        public LeftScaleMacro () {
            super(LeftScaleMacro.class);

            addState("Grab preloaded cube", new GrabPreloadedCubeMacro());

            addState("Drive away from wall", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(222+1)), 0);

                @Override
                protected boolean run () {
                    robot.elevator.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Continue based on game data", new LeftScaleDecisionMacro());
        }
    }

    private class RightScaleMacro extends StatefulCoordinator {
        public RightScaleMacro () {
            super(RightScaleMacro.class);

            addState("Grab preloaded cube", new GrabPreloadedCubeMacro());

            addState("Drive away from wall", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(222+1)), 0);

                @Override
                protected boolean run () {
                    robot.elevator.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Continue based on game data", new RightScaleDecisionMacro());
        }
    }

    private class LeftScaleSameOnlyMacro extends StatefulCoordinator {
        public LeftScaleSameOnlyMacro() {
            super(LeftScaleSameOnlyMacro.class);

            addState("Grab preloaded cube", new GrabPreloadedCubeMacro());

            addState("Drive away from wall", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(222+1)), 0);

                @Override
                protected boolean run () {
                    robot.elevator.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Continue based on game data", new LeftScaleSameOnlyDecisionMacro());
        }
    }

    private class RightScaleSameOnlyMacro extends StatefulCoordinator {
        public RightScaleSameOnlyMacro() {
            super(RightScaleSameOnlyMacro.class);

            addState("Grab preloaded cube", new GrabPreloadedCubeMacro());

            addState("Drive away from wall", new Coordinator() {
                private final Drive.ArcadeServo arcadeServo = robot.drive.new ArcadeServo(
                        AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(222+1)), 0);

                @Override
                protected boolean run () {
                    robot.elevator.mid.activate();

                    arcadeServo.activate();
                    return arcadeServo.onTarget.get();
                }
            });

            addState("Continue based on game data", new RightScaleSameOnlyDecisionMacro());
        }
    }

    private class LeftScaleDecisionMacro extends SwitchCoordinator {
        public LeftScaleDecisionMacro () {
            super(LeftScaleDecisionMacro.class);
            addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new NewScaleBackwardLeftMacro());
            addCase(new String[]{"LRL", "LRR", "RRL", "RRR"}, new NewScaleOppositeLeftMacro());
        }
    }

    private class RightScaleDecisionMacro extends SwitchCoordinator {
        public RightScaleDecisionMacro () {
            super(RightScaleDecisionMacro.class);
            addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new NewScaleOppositeRightMacro());
            addCase(new String[]{"LRL", "LRR", "RRL", "RRR"}, new NewScaleBackwardRightMacro());
        }
    }

    private class LeftScaleSameOnlyDecisionMacro extends SwitchCoordinator {
        public LeftScaleSameOnlyDecisionMacro () {
            super(LeftScaleSameOnlyDecisionMacro.class);
            addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new NewScaleBackwardLeftMacro());
            addCase(new String[]{"LRL", "LRR", "RRL", "RRR"}, new HoldElevatorArmMacro());
        }
    }

    private class RightScaleSameOnlyDecisionMacro extends SwitchCoordinator {
        public RightScaleSameOnlyDecisionMacro () {
            super(RightScaleSameOnlyDecisionMacro.class);
            addCase(new String[]{"LLL", "LLR", "RLL", "RLR"}, new HoldElevatorArmMacro());
            addCase(new String[]{"LRL", "LRR", "RRL", "RRR"}, new NewScaleBackwardRightMacro());
        }
    }

    private class HoldElevatorArmMacro extends Coordinator {
        @Override
        protected boolean run () {
            robot.elevator.hold.activate();
            robot.arm.hold.activate();
            return true;
        }
    }

    private class NewScaleBackwardLeftMacro extends StatefulCoordinator {
    	public NewScaleBackwardLeftMacro () {
    		super(NewScaleBackwardLeftMacro.class);

    		addState("Grab preloaded cube", new GrabPreloadedCubeMacro());

            addState("Drive away from wall", new DriveArcadeServoMacro(
                    AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(36+1)), 0));

            addState("Turn toward scale", new DriveArcadeServoMacro(
                    0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 35)));

            addState("Drive toward scale", new DriveArcadeServoMacro(
                    AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(9)), 0));

            addState("Raise arm", new TimeLimitCoordinator(1.3, new ActionCoordinator(robot.arm.high)));

            addState("Eject cube", new TimeLimitCoordinator(0.5, new Coordinator() {
                private final Intake.Run runIntake = robot.intake.new Run(-0.5);

                @Override
                protected boolean run () {
                    robot.arm.high.activate();
                    runIntake.run();
                    return true;
                }
            }));

            addState("Disengage and wait", new Coordinator() {
                @Override
                protected boolean run () {
                    robot.elevator.low.activate();
                    robot.arm.low.activate();
                    robot.clamp.release.activate();
                    return true;
                }
            });
    	}
    }

    private class NewScaleBackwardRightMacro extends StatefulCoordinator {
        public NewScaleBackwardRightMacro () {
            super(NewScaleBackwardRightMacro.class);

            addState("Grab preloaded cube", new GrabPreloadedCubeMacro());

            addState("Drive away from wall", new DriveArcadeServoMacro(
                    AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(36+1)), 0));

            addState("Turn toward scale", new DriveArcadeServoMacro(
                    0, AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, -35)));

            addState("Drive toward scale", new DriveArcadeServoMacro(
                    AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, -(9)), 0));

            addState("Raise arm", new TimeLimitCoordinator(1.3, new ActionCoordinator(robot.arm.high)));

            addState("Eject cube", new TimeLimitCoordinator(0.5, new Coordinator() {
                private final Intake.Run runIntake = robot.intake.new Run(-0.5);

                @Override
                protected boolean run () {
                    robot.arm.high.activate();
                    runIntake.run();
                    return true;
                }
            }));

            addState("Disengage and wait", new Coordinator() {
                @Override
                protected boolean run () {
                    robot.elevator.low.activate();
                    robot.arm.low.activate();
                    robot.clamp.release.activate();
                    return true;
                }
            });
        }
    }


    private class NewScaleOppositeLeftMacro extends StatefulCoordinator {
        public NewScaleOppositeLeftMacro () {
            super(NewScaleOppositeLeftMacro.class);
            //addStates(new GrabPreloadedCubeMacro());
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
            addState("Engage arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
            addState("Engage elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
            addState("Unclamp", new ClampExtend());
        }
    }
    
    private class NewScaleOppositeRightMacro extends StatefulCoordinator {
        public NewScaleOppositeRightMacro () {
            super(NewScaleOppositeRightMacro.class);
          //addStates(new GrabPreloadedCubeMacro());
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
            addState("Engage arm", new ArmSetPersistent(Calibration.ARM_LOW_TARGET));
            addState("Engage elevator", new ElevatorSetPersistent(Calibration.ELEVATOR_LOW_TARGET));
            addState("Unclamp", new ClampExtend());
        }
    }
}
