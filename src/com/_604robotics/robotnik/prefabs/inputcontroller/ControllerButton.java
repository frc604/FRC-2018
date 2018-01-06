package com._604robotics.robotnik.prefabs.inputcontroller;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A button on a controller.
 */
public class ControllerButton {
    private final Joystick joystick;
    private final int button;

    /**
     * Creates a controller button.
     * @param joystick Joystick containing the button.
     * @param button Button to represent.
     */
    public ControllerButton (Joystick joystick, int button) {
        this.joystick = joystick;
        this.button = button;
    }

    public boolean get () {
        return this.joystick.getRawButton(this.button);
    }
}