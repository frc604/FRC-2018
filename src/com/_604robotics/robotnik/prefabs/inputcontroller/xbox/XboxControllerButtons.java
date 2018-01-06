package com._604robotics.robotnik.prefabs.inputcontroller.xbox;

import com._604robotics.robotnik.prefabs.inputcontroller.ControllerAxisButton;
import com._604robotics.robotnik.prefabs.inputcontroller.ControllerButton;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The buttons of an Xbox controller.
 */
public class XboxControllerButtons {
    /**
     * A.
     */
    public final ControllerButton a;

    /**
     * B.
     */
    public final ControllerButton b;

    /**
     * X.
     */
    public final ControllerButton x;

    /**
     * Y.
     */
    public final ControllerButton y;

    /**
     * LB.
     */
    public final ControllerButton lb;

    /**
     * RB.
     */
    public final ControllerButton rb;

    /**
     * Back.
     */
    public final ControllerButton back;

    /**
     * Left Stick.
     */
    public final ControllerButton leftStick;

    /**
     * Right Stick.
     */
    public final ControllerButton rightStick;

    /**
     * LT.
     */
    public final ControllerAxisButton lt;

    /**
     * RT.
     */
    public final ControllerAxisButton rt;

    /**
     * Start.
     */
    public final ControllerButton start;

    /**
     * Creates Xbox controller buttons.
     * @param joystick Xbox controller containing the buttons.
     */
    public XboxControllerButtons (Joystick joystick) {
        this.a = new ControllerButton(joystick, 1);
        this.b = new ControllerButton(joystick, 2);
        this.x = new ControllerButton(joystick, 3);
        this.y = new ControllerButton(joystick, 4);
        this.lb = new ControllerButton(joystick, 5);
        this.rb = new ControllerButton(joystick, 6);
        this.back = new ControllerButton(joystick, 7);
        this.start = new ControllerButton(joystick, 8);

        this.leftStick = new ControllerButton(joystick,  9);
        this.rightStick = new ControllerButton(joystick, 10);

        this.lt = new ControllerAxisButton(joystick, 2, 1);
        this.rt = new ControllerAxisButton(joystick, 3, 1);
    }
}