package com._604robotics.robotnik.prefabs.coordinators;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class GameDataCoordinator extends Coordinator {
    private String gameData = "";
    private Timer timeout = new Timer();
    private Logger logGameData = new Logger(GameDataCoordinator.class);
    
    @Override
    protected void begin() {
        timeout.start();
    }

    @Override
    protected boolean run() {
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        return (gameData==null) || (gameData.length()==0) || (timeout.get()<1);
    }

    @Override
    protected void end() {
        logGameData.info("Game data is "+gameData);
        timeout.reset();
    }

    public GameDataCoordinator() {
        // Do nothing
    }
    
    public String getGameData() {
        if (gameData==null || gameData.length()==0) {
            logGameData.info("Attempted to get Game Data while it was not ready!");
        }
        return gameData;
    }

}
