package com._604robotics.robotnik.prefabs.devices;

/**
 * A pair of ultrasonic sensors.
 */
public class UltrasonicPair implements Ultrasonic {
    private Ultrasonic left;
    private Ultrasonic right;
    private double separation;

    /**
     * Creates a new ultrasonic pair.
     * @param left Left sensor.
     * @param right Right sensor.
     * @param separation Distance between the sensors, in inches.
     */
    public UltrasonicPair (Ultrasonic left, Ultrasonic right, double separation) {
        this.left = left;
        this.right = right;
        this.separation = separation;
    }

    /**
     * Gets the difference in the distances read by the sensors using 64 samples.
     * @return The difference in the distances read by the sensors.
     */
    public double getDifference () {
        return this.right.getDistance() - this.left.getDistance();
    }

    /**
     * Gets the difference in the distances read by the sensors.
     * @param samples Number of samples to take.
     * @return The difference in the distances read by the sensors.
     */
    public double getDifference (int samples) {
        return this.right.getDistance(samples) - this.left.getDistance(samples);
    }

    /**
     * Gets the angle of the sensors relative to a wall using 64 samples. To the left is negative, straight-on is zero, and to the right is positive.
     * @return The angle of the sensors relative to a wall.
     */
    public double getAngle () {
        double radians = Math.atan(getDifference()/separation);
        double angle = radians * 180 / Math.PI;
        if (getDifference() < 0) {
            return angle * -1;
        } else {
            return angle;
        }
    }

    /**
     * Gets the angle of the sensors relative to a wall. To the left is negative, straight-on is zero, and to the right is positive.
     * @param samples Number of samples to take.
     * @return The angle of the sensors relative to a wall.
     */
    public double getAngle (int samples) {
        double angle = Math.acos(this.separation / Math.pow((Math.pow(this.separation, 2) + Math.pow(getDifference(samples), 2)), 0.5));
        if (this.getDifference(samples) < 0) {
            return angle * -1;
        } else {
            return angle;
        }
    }

    /**
     * Gets the direction that the sensors are facing relative to a wall.
     * @return The direction that the sensors are facing relative to a wall.
     */
    public String getDirection () {
        if (this.getDifference() < 0) {
            return "left";
        } else if (this.getDifference() > 0) {
            return "right";
        } else {
            return "straight";
        }
    }

    /**
     * Gets the direction that the sensors are facing relative to a wall.
     * @param tolerance Tolerance value used to differentiate left and right from straight-on.
     * @return The direction that the sensors are facing relative to a wall.
     */
    public String getDirection (double tolerance) {
        if (this.getDifference() < -tolerance) {
            return "left";
        } else if (this.getDifference() > tolerance) {
            return "right";
        } else {
            return "straight";
        }
    }

    /**
     * Gets the distance read by the left sensor using 64 samples.
     * @return The distance read by the left sensor.
     */
    public double getLeftDistance () {
        return this.left.getDistance();
    }

    /**
     * Gets the distance read by the left sensor.
     * @param samples Number of samples to take.
     * @return The distance read by the left sensor.
     */
    public double getLeftDistance (int samples) {
        return this.left.getDistance(samples);
    }

    /**
     * Gets the distance read by the right sensor using 64 samples.
     * @return The distance read by the right sensor.
     */
    public double getRightDistance () {
        return this.right.getDistance();
    }

    /**
     * Gets the distance read by the right sensor.
     * @param samples Number of samples to take.
     * @return The distance read by the right sensor.
     */
    public double getRightDistance (int samples) {
        return this.right.getDistance(samples);
    }

    @Override
    public double getDistance () {
        double leftDistance = this.left.getDistance();
        double rightDistance = this.right.getDistance();
        return Math.min(leftDistance, rightDistance);
    }

    @Override
    public double getDistance (int samples) {
        double leftDistance = this.left.getDistance(samples);
        double rightDistance = this.right.getDistance(samples);
        return Math.min(leftDistance, rightDistance);
    }

    @Override
    public boolean inRange () {
        return this.left.inRange() && this.right.inRange();
    }
}