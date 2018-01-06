package com._604robotics.robotnik.prefabs.inputcontroller.wheel;

import com._604robotics.robotnik.prefabs.inputcontroller.ControllerButton;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The buttons of a wheel controller.
 */
public class WheelControllerButtons {
    /**
     * Left Pad.
     */
    public final ControllerButton leftPad;

    /**
     * Right Pad.
     */
    public final ControllerButton rightPad;

    /**
     * Triangle.
     */
    public final ControllerButton triangle;

    /**
     * Square.
     */
    public final ControllerButton square;

    /**
     * Circle.
     */
    public final ControllerButton circle;

    /**
     * Cross.
     */
    public final ControllerButton cross;

    /**
     * L2.
     */
    public final ControllerButton l2;

    /**
     * R2.
     */
    public final ControllerButton r2;

    /**
     * L3.
     */
    public final ControllerButton l3;

    /**
     * R3.
     */
    public final ControllerButton r3;

    /**
     * Home.
     */
    public final ControllerButton home;

    /**
     * ST.
     */
    public final ControllerButton st;

    /**
     * SE.
     */
    public final ControllerButton se;

    /**
     * Creates wheel controller buttons.
     * @param joystick Wheel controller containing the buttons.
     */
    public WheelControllerButtons (Joystick joystick) {
        this.leftPad = new ControllerButton(joystick, 5);
        this.rightPad = new ControllerButton(joystick, 6);

        this.triangle = new ControllerButton(joystick, 4);
        this.square = new ControllerButton(joystick, 1);
        this.circle = new ControllerButton(joystick, 3);
        this.cross = new ControllerButton(joystick, 2);

        this.l2 = new ControllerButton(joystick, 7);
        this.r2 = new ControllerButton(joystick, 8);

        this.l3 = new ControllerButton(joystick, 11);
        this.r3 = new ControllerButton(joystick, 12);

        this.home = new ControllerButton(joystick, 13);
        this.st = new ControllerButton(joystick, 10);
        this.se = new ControllerButton(joystick, 9);
    }
}