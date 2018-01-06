package com._604robotics.robotnik.prefabs.inputcontroller.joystick;

import com._604robotics.robotnik.prefabs.inputcontroller.ControllerAxis;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A joystick controller.
 */
public class JoystickController {
    /**
     * The joystick's buttons.
     */
    public final JoystickControllerButtons buttons;

    /**
     * The joystick's X axis.
     */
    public final ControllerAxis axisX;

    /**
     * The joystick's Y axis.
     */
    public final ControllerAxis axisY;

    /**
     * The joystick's adjust axis.
     */
    public final ControllerAxis axisAdjust;

    /**
     * Creates a joystick controller.
     * @param port Port of the controller.
     */
    public JoystickController (int port) {
        final Joystick joystick = new Joystick(port);

        buttons = new JoystickControllerButtons(joystick);

        axisX = new ControllerAxis(joystick, 0);
        axisY = new ControllerAxis(joystick, 1);
        axisAdjust = new ControllerAxis(joystick, 2);
    }
}