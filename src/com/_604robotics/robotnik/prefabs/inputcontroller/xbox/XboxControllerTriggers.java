package com._604robotics.robotnik.prefabs.inputcontroller.xbox;

import com._604robotics.robotnik.prefabs.inputcontroller.ControllerAxis;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The triggers of an Xbox controller.
 */
public class XboxControllerTriggers {
    /**
     * Left.
     */
    public final ControllerAxis left;

    /**
     * Right.
     */
    public final ControllerAxis right;

    /**
     * Creates Xbox controller triggers.
     * @param joystick Xbox controller containing the triggers.
     * @param left Left trigger ID.
     * @param right Right trigger ID.
     */
    public XboxControllerTriggers (Joystick joystick, int left, int right) {
        this.left = new ControllerAxis(joystick, left);
        this.right = new ControllerAxis(joystick, right);
    }
}