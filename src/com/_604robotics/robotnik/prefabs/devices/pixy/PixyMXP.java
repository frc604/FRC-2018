package com._604robotics.robotnik.prefabs.devices.pixy;

// Credit goes to team 1559

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class PixyMXP {
	public final SerialPort pixy;
	public final Port port = Port.kMXP;
	public int objectID;

	private PixyPacketMXP[] packets;
	
	public PixyMXP() {
		pixy = new SerialPort(19200, port);
		pixy.setReadBufferSize(14);
		packets = new PixyPacketMXP[7];
		objectID = 1;
	}

	//This method parses raw data from the pixy into readable integers
	private int cvt(byte upper, byte lower) {
		return (((int)upper & 0xff) << 8) | ((int)lower & 0xff);
	}

	public void pixyReset(){
		pixy.reset();
	}
	
	public void update() {
		int Checksum;
		int Sig;
		byte[] rawData = new byte[32];
		try{
			rawData = pixy.read(32);
		} catch (RuntimeException e){
			System.out.println("Runtime exception reading from pixy");
		}
		if(rawData.length < 32){
			System.out.println("byte array length is broken");
		} else {
			for (int i = 0; i <= 16; i++) {
				int syncWord = cvt(rawData[i+1], rawData[i+0]); //Parse first 2 bytes
				if (syncWord == 0xaa55) { //Check is first 2 bytes equal a "sync word", which indicates the start of a packet of valid data
					syncWord = cvt(rawData[i+3], rawData[i+2]); //Parse the next 2 bytes
					if (syncWord != 0xaa55){ //Shifts everything in the case that one syncword is sent
						i -= 2;
					}
					//This next block parses the rest of the data
					Checksum = cvt(rawData[i+5], rawData[i+4]);
					Sig = cvt(rawData[i+7], rawData[i+6]);
					if(Sig <= 0 || Sig > packets.length){
						break;
					}
					packets[Sig - 1] = new PixyPacketMXP();
					packets[Sig - 1].X = cvt(rawData[i+9], rawData[i+8]);
					packets[Sig - 1].Y = cvt(rawData[i+11], rawData[i+10]);
					packets[Sig - 1].Width = cvt(rawData[i+13], rawData[i+12]);
					packets[Sig - 1].Height = cvt(rawData[i+15], rawData[i+14]);
					//Checks whether the data is valid using the checksum *This if block should never be entered*
					if (Checksum != Sig + packets[Sig - 1].X + packets[Sig - 1].Y + packets[Sig - 1].Width + packets[Sig - 1].Height) {
						packets[Sig - 1] = null;
					}
					break;
				}
			}
		}
	}
	
	// probably not the best way to go about this
	public void setSignature( int objectID ) {
		this.objectID = objectID;
	}
	
	public double getX() {
		return readPacket().X;
	}
	public double getY() {
		return readPacket().Y;
	}
	public double getHeight() {
		return readPacket().Height;
	}
	public double getWidth() {
		return readPacket().Width;
	}
	
	public PixyPacketMXP readPacket() {
		return packets[objectID-1];
	}
}