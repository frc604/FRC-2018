package com._604robotics.marionette;

import edu.wpi.first.wpilibj.Joystick;

public class MarionetteJoystick extends Joystick {
    private final InputPlayer player;
    private final int index;

    public MarionetteJoystick (final int port, final InputPlayer player, final int index) {
        super(port);
        this.player = player;
        this.index = index;
    }

    @Override
    public int getAxisCount () {
        return player.getAxisCount(index, super.getAxisCount());
    }

    @Override
    public int getButtonCount () {
        return player.getButtonCount(index, super.getButtonCount());
    }

    @Override
    public double getRawAxis (final int axis) {
        return player.getRawAxis(index, axis, super.getRawAxis(axis));
    }

    @Override
    public boolean getRawButton (final int button) {
        return player.getRawButton(index, button, super.getRawButton(button));
    }
}