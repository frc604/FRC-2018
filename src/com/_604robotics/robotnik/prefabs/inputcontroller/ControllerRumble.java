package com._604robotics.robotnik.prefabs.inputcontroller;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The rumble feature of a controller.
 */
public class ControllerRumble {
    private final Joystick joystick;
    private boolean enabled;
    private float leftRumble;
    private float rightRumble;

    /**
     * Creates a controller rumble.
     * @param joystick Joystick containing the rumble feature.
     */
    public ControllerRumble (final Joystick joystick) {
        this.joystick = joystick;
        this.enabled = false;
        this.leftRumble = 0;
        this.rightRumble = 0;
    }

    public boolean isEnabled () {
        return enabled;
    }

    public void setEnabled (boolean enabled) {
        this.enabled = enabled;
        update();
    }

    public void enable () {
        setEnabled(true);
    }

    public void disable () {
        setEnabled(false);
    }

    public float getLeftRumble () {
        return leftRumble;
    }

    public float getRightRumble () {
        return rightRumble;
    }

    public void setRumble (float rumble) {
        setRumble(rumble, rumble);
    }

    public void setRumble (float leftRumble, float rightRumble) {
        this.leftRumble = leftRumble;
        this.rightRumble = rightRumble;
        update();
    }

    private void update () {
        if (isEnabled()) {
            joystick.setRumble(Joystick.RumbleType.kLeftRumble, getLeftRumble());
            joystick.setRumble(Joystick.RumbleType.kRightRumble, getRightRumble());
        } else {
            joystick.setRumble(Joystick.RumbleType.kLeftRumble, 0);
            joystick.setRumble(Joystick.RumbleType.kRightRumble, 0);
        }
    }
}