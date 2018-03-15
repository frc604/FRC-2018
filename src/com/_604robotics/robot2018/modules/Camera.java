package com._604robotics.robot2018.modules;

import com._604robotics.robotnik.Module;

import edu.wpi.first.wpilibj.CameraServer;

public class Camera extends Module {

    public Camera() {
        super(Camera.class);
        CameraServer.getInstance().startAutomaticCapture("cam0", "/dev/video0");
    }
    
    

}
