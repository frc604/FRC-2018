package com._604robotics.robotnik.prefabs.devices;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Analog ultrasonic sensor.
 */
public class AnalogUltrasonic extends AnalogInput implements Ultrasonic {
    private static final int DEFAULT_SAMPLES = 64;
    private static final double INCHES_PER_VOLT = 42.56;
    private static final double OPERATING_RANGE = 150.0;

    /**
     * Creates an analog ultrasonic sensor.
     * @param port Analog port of the sensor.
     */
    public AnalogUltrasonic (int port) {
        super(port);
    }

    /**
     * Gets the current analog value of the sensor using 64 samples.
     * @return The current analog value of the sensor.
     */
    public double getAnalog () {
        return this.getAnalog(DEFAULT_SAMPLES);
    }

    /**
     * Gets the current analog value of the sensor.
     * @param samples Number of samples to take.
     * @return The current analog value of the sensor.
     */
    public double getAnalog (int samples) {
        double total = 0;
        for (int f = 0; f < samples; f++) {
            total += super.getValue();
        }

        total /= samples;
        return total;
    }

    /**
     * Gets the current voltage of the sensor using 64 samples.
     * @return The current voltage of the sensor.
     */
    @Override
    public double getVoltage () {
        return this.getVoltage(DEFAULT_SAMPLES);
    }

    /**
     * Gets the current voltage of the sensor.
     * @param samples Number of samples to take.
     * @return The current voltage of the sensor.
     */
    public double getVoltage (int samples) {
        double total = 0;
        for (int f = 0; f < samples; f++) {
            total += super.getVoltage();
        }

        total /= samples;
        return total;
    }

    @Override
    public double getDistance () {
        return this.getVoltage() * INCHES_PER_VOLT;
    }

    @Override
    public double getDistance (int samples) {
        return this.getVoltage(samples) * INCHES_PER_VOLT;
    }

    @Override
    public boolean inRange () {
        return this.getDistance(1) < OPERATING_RANGE;
    }
}