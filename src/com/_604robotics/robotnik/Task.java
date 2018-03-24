package com._604robotics.robotnik;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public final class Task <V> implements AutoCloseable { 
	private ExecutorService exe;
	private Future<V> future;
	
	public Task( Callable<V> callable ) {
		exe = Executors.newSingleThreadExecutor();
		future = exe.submit( callable );
	}
	
	public void cancel() {
		future.cancel( true );
	}
	
	public boolean isReady() {
		return future.isDone();
	}
	
	public V get() throws InterruptedException, ExecutionException {
		return future.get();
	}
	
	public V getIfReady() throws InterruptedException, ExecutionException {
		if( isReady() ) {
			return get();
		}
		
		return null;
	}
	
	@Override
    public void close() {
        if (exe!=null && !exe.isShutdown()) {
            exe.shutdown();
        }
    }
}
