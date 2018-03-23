package com._604robotics.robot2018.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.prefabs.devices.pixy.PixyException;
import com._604robotics.robotnik.prefabs.devices.pixy.PixyI2C;
import com._604robotics.robotnik.prefabs.devices.pixy.PixyPacketI2C;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class PixyTest extends Module {
	// private PixyMXP pixy = new PixyMXP();
	public String print;
	public PixyPacketI2C[] packet = new PixyPacketI2C[7];
	public PixyI2C pixy = new PixyI2C("Pixy", new I2C(Port.kOnboard, 0x54), packet, new PixyException(print), new PixyPacketI2C());
	
	public Action stream = new Stream();
	
	public class Stream extends Action {
		public Stream() {
			super(PixyTest.this, Stream.class);
		}
		
		@Override
		public void run() {
			pixy.readBlocks();
		}
	}
	
	public PixyTest() {
		super(PixyTest.class);
		this.setDefaultAction(stream);
	}
}
