package com._604robotics.robotnik.prefabs.devices;

/**
 * An ultrasonic sensor.
 */
public interface Ultrasonic {
    /**
     * Gets the current distance measured by the sensor, in inches, using 64 samples.
     * @return The distance, in inches.
     */
    public double getDistance ();

    /**
     * Gets the current distance measured by the sensor, in inches.
     * @param samples Number of samples to take.
     * @return The distance, in inches.
     */
    public double getDistance (int samples);

    /**
     * Determines if the sensor is within its operating range.
     * @return Whether or not the sensor is within its operating range.
     */
    public boolean inRange ();
}