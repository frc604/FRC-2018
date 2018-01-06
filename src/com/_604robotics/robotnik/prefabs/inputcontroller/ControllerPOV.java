package com._604robotics.robotnik.prefabs.inputcontroller;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A POV-hat on a controller.
 */
public class ControllerPOV {
    private final Joystick joystick;
    private final int port;

    /**
     * Creates a controller POV.
     * @param joystick Joystick containing the POV.
     * @param port Port of the POV.
     */
    public ControllerPOV (Joystick joystick, int port) {
        this.joystick = joystick;
        this.port = port;
    }

    public double get () {
        return this.joystick.getPOV(this.port);
    }
}