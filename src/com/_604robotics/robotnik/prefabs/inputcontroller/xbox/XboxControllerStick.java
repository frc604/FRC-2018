package com._604robotics.robotnik.prefabs.inputcontroller.xbox;

import com._604robotics.robotnik.prefabs.inputcontroller.ControllerAxis;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A stick on an Xbox controller.
 */
public class XboxControllerStick {
    /**
     * X-axis.
     */
    public final ControllerAxis x;

    /**
     * Y-axis.
     */
    public final ControllerAxis y;

    /**
     * Creates an Xbox controller stick.
     * @param joystick Xbox controller containing the stick.
     * @param x X-axis ID.
     * @param y Y-axis ID.
     */
    public XboxControllerStick (Joystick joystick, int x, int y) {
        this.x = new ControllerAxis(joystick, x);
        this.y = new ControllerAxis(joystick, y);
    }
}