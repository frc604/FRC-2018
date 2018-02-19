package com._604robotics.robotnik.prefabs.coordinators;

import com._604robotics.robotnik.Coordinator;

import edu.wpi.first.wpilibj.DriverStation;

public class GameDataCoordinator extends Coordinator {
    private String gameData = "";
    
    @Override
    protected void begin() {
        // Do nothing
    }

    @Override
    protected boolean run() {
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        return (gameData!=null) && (gameData.length()>0);
    }

    @Override
    protected void end() {
        //Do nothing
    }

    public GameDataCoordinator() {
        // TODO Auto-generated constructor stub
    }
    
    public String getGameData() {
        return gameData;
    }

}
