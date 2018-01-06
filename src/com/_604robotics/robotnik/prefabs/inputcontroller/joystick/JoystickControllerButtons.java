package com._604robotics.robotnik.prefabs.inputcontroller.joystick;

import com._604robotics.robotnik.prefabs.inputcontroller.ControllerButton;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The buttons of a joystick controller.
 */
public class JoystickControllerButtons {
    /**
     * Button 1.
     */
    public final ControllerButton button1;

    /**
     * Button 2.
     */
    public final ControllerButton button2;

    /**
     * Button 3.
     */
    public final ControllerButton button3;

    /**
     * Button 4.
     */
    public final ControllerButton button4;

    /**
     * Button 5.
     */
    public final ControllerButton button5;

    /**
     * Button 6.
     */
    public final ControllerButton button6;

    /**
     * Button 7.
     */
    public final ControllerButton button7;

    /**
     * Button 8.
     */
    public final ControllerButton button8;

    /**
     * Button 9.
     */
    public final ControllerButton button9;

    /**
     * Button 10.
     */
    public final ControllerButton button10;

    /**
     * Button 11.
     */
    public final ControllerButton button11;

    /**
     * Creates joystick controller buttons.
     * @param joystick Joystick containing the buttons.
     */
    public JoystickControllerButtons (Joystick joystick) {
        button1  = new ControllerButton(joystick, 1);
        button2  = new ControllerButton(joystick, 2);
        button3  = new ControllerButton(joystick, 3);
        button4  = new ControllerButton(joystick, 4);
        button5  = new ControllerButton(joystick, 5);
        button6  = new ControllerButton(joystick, 6);
        button7  = new ControllerButton(joystick, 7);
        button8  = new ControllerButton(joystick, 8);
        button9  = new ControllerButton(joystick, 9);
        button10 = new ControllerButton(joystick, 10);
        button11 = new ControllerButton(joystick, 11);
    }
}