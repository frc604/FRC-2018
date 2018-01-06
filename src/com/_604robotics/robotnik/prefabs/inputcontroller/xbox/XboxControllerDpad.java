package com._604robotics.robotnik.prefabs.inputcontroller.xbox;

import com._604robotics.robotnik.prefabs.inputcontroller.ControllerPOV;
import com._604robotics.robotnik.prefabs.inputcontroller.ControllerPOVButton;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The D-pad of an Xbox controller.
 */
public class XboxControllerDpad {
    /**
     * Pad POV-hat.
     */
    public final ControllerPOV pov;

    /**
     * Up.
     */
    public final ControllerPOVButton up;

    /**
     * Down.
     */
    public final ControllerPOVButton down;

    /**
     * Left.
     */
    public final ControllerPOVButton left;

    /**
     * Right.
     */
    public final ControllerPOVButton right;

    /**
     * Pressed.
     */
    public final ControllerPOVButton pressed;

    /**
     * Imprecise D-pad measurements.
     */
    public class XboxControllerDpadImprecise {
        /**
         * Up.
         */
        public final ControllerPOVButton up;

        /**
         * Down.
         */
        public final ControllerPOVButton down;

        /**
         * Left.
         */
        public final ControllerPOVButton left;

        /**
         * Right.
         */
        public final ControllerPOVButton right;

        public XboxControllerDpadImprecise (ControllerPOVButton up, ControllerPOVButton down, ControllerPOVButton left,
                                            ControllerPOVButton right) {
            this.up = up;
            this.down = down;
            this.left = left;
            this.right = right;
        }
    }

    /**
     * Imprecise D-pad measurements.
     */
    public final XboxControllerDpadImprecise imprecise;

    /**
     * Creates an Xbox controller D-pad.
     * @param joystick Xbox controller containing the D-pad.
     */
    public XboxControllerDpad (Joystick joystick){
        this.pov = new ControllerPOV(joystick, 0);
        this.up = new ControllerPOVButton(joystick, 0, 0);
        this.down = new ControllerPOVButton(joystick, 0, 180);
        this.left = new ControllerPOVButton(joystick, 0, 270);
        this.right = new ControllerPOVButton(joystick, 0, 90);
        this.pressed = new ControllerPOVButton(joystick, 0, 0, 360);
        this.imprecise = new XboxControllerDpadImprecise(
                new ControllerPOVButton(joystick, 0, 315, 45),
                new ControllerPOVButton(joystick, 0, 135, 225),
                new ControllerPOVButton(joystick, 0, 45, 135),
                new ControllerPOVButton(joystick, 0, 225, 315)
        );
    }
}