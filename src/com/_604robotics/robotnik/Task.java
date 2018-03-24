package com._604robotics.robotnik;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public abstract class Task <V> implements Callable<V>, AutoCloseable { 
	private V result;
	private ExecutorService exe;
	private Future<V> future;
	
	public Task() {
		exe = Executors.newSingleThreadExecutor();
	}
	
}
