package com._604robotics.robotnik.prefabs.devices.pixy;

public class PixyPacketI2C {
	public int Signature;
	public int X;
	public int Y;
	public int Width;
	public int Height;
	
	//public int checksumError;
	
	public String toString() {
		return "" +
	" S:" + Signature +
	" X:" + X + 
	" Y:" + Y +
	" W:" + Width + 
	" H:" + Height;
	}
}
