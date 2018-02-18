package com._604robotics.robotnik.utils;

public class AutonMovement {

	private AutonMovement() {}
	
	public static double clicksToDegrees(DriveTrainProperties properties, double clicks) {
		return clicks*(properties.getDegreesOverClicks());
	}
	
	public static double degreesToClicks(DriveTrainProperties properties, double degrees) {
		return degrees/(properties.getDegreesOverClicks());
	}
	
	public static double inchesToClicks(DriveTrainProperties properties, double inches) {
		return inches*(properties.getClicksOverInches());
	}
	
	public static double clicksToInches(DriveTrainProperties properties, double clicks) {
		return clicks/(properties.getClicksOverInches());
	}
	
	public static double empericalClicksToInches(DriveTrainProperties properties, double clicks) {
		return properties.empericalClicksToInches(clicks);
	}
	
	public static double empericalInchesToClicks(DriveTrainProperties properties, double inches) {
		return properties.empericalInchesToClicks(inches);
	}

	/**
	 * A class that represents a Drive Train.
	 */
	public static class DriveTrainProperties {
		
		public final double clicksPerRot;
		public final double width;
		public final double wheelRadius;
		public final double clicksPerInches;
		public final double offset;
		
		/**
		 * Instantiates a DriveTrainProperties
		 * @param clicksPerRot Encoder clicks per full rotation of wheel
		 * @param width Width of the robot in inches (may need empirical calibration of +- 0.3 in)
		 * @param wheelRadius Radius of robot wheels in inches
		 * @param clicksPerInches Empirical clicks per inches ratio
		 * @param offset Empirical offset of clicks for the robot
		 */
		public DriveTrainProperties(double clicksPerRot, double width, double wheelRadius, double clicksPerInches, double offset) {
			this.clicksPerRot=clicksPerRot;
			this.width=width;
			this.wheelRadius=wheelRadius;
			this.offset = offset;
			this.clicksPerInches = clicksPerInches;
			if (clicksPerInches<0) {
			    clicksPerInches = getClicksOverInches();
			}
		}
		
		public double getDegreesOverClicks() {
			double conversionFactor = (2*wheelRadius)/width;
			conversionFactor*=(2);//*3.14159265359);
			conversionFactor/=clicksPerRot;
			conversionFactor*=(180);///3.14159265359);
			return conversionFactor;
		}
		
		public double getClicksOverInches() {
			double conversionFactor = clicksPerRot;
			conversionFactor/=(2*Math.PI);
			conversionFactor/=wheelRadius;
			// Un-understood empirical factor of 2
			conversionFactor/=2;
			return conversionFactor;
		}
		
		public double empericalClicksToInches(double clicks) {
			return (clicks + offset) / clicksPerInches;
		}
		
		public double empericalInchesToClicks(double inches) {
			return clicksPerInches * inches - offset;
		}
	}
}