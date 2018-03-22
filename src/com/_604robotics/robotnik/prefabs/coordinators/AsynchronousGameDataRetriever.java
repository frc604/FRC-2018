package com._604robotics.robotnik.prefabs.coordinators;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import edu.wpi.first.wpilibj.DriverStation;

public final class AsynchronousGameDataRetriever implements AutoCloseable {
    private final ExecutorService executor;
    private final Future<String> future;
    private String result;
    
    private final class GameDataRetriever implements Callable<String> {

        @Override
        public String call(){
            String result;
            do {
                result = DriverStation.getInstance().getGameSpecificMessage();
            } while (result==null || result.length()==0);
            return result;
        }
    }
    
    public AsynchronousGameDataRetriever() {
        executor = Executors.newSingleThreadExecutor();
        future=executor.submit(new GameDataRetriever());
    }
    
    public boolean isReady() {
        return future.isDone();
    }
    public String get() throws InterruptedException, ExecutionException {
        if (isReady()) {
            result = future.get();
        }
        return result;
    }
    
    public String getIfReady() throws InterruptedException, ExecutionException {
        try {
            return future.get(50,TimeUnit.MILLISECONDS);
        } catch (TimeoutException e) {
            return null;
        }
    }

    @Override
    public void close() {
        if (executor!=null && !executor.isShutdown()) {
            executor.shutdown();
        }
    }
}
