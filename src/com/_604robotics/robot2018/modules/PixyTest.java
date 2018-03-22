package com._604robotics.robot2018.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.devices.pixy.Pixy;

public class PixyTest extends Module {
	private Pixy pixy = new Pixy();
	
	private double x;
	private double y;
	private double height;
	private double width;
	
	public Output<Double> objectX = addOutput("Object X", this::getX);
	public Output<Double> objectY = addOutput("Object Y", this::getY);
	public Output<Double> objectHeight = addOutput("Object Height", this::getHeight);
	public Output<Double> objectWidth = addOutput("Object Width", this::getWidth);
	
	public Action stream = new Stream();
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public double getHeight() {
		return height;
	}
	
	public double getWidth() {
		return width;
	}
	
	public class Stream extends Action {
		public Stream() {
			super(PixyTest.this, Stream.class);
		}
		
		@Override
		public void run() {
			pixy.update();
			x = pixy.getX();
			y = pixy.getY();
			height = pixy.getHeight();
			width = pixy.getWidth();
		}
	}
	
	public PixyTest() {
		super(PixyTest.class);
		this.setDefaultAction(stream);
	}
}
