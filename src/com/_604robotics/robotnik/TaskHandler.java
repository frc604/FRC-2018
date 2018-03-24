package com._604robotics.robotnik;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class TaskHandler {
	/*
	 * We store the refrence to each callable by the variable it will be stored it.
	 * We want the ability to have multiple callables in one TaskHandler
	 * This allows us to group related tasks together, abstracting that here instead of the caller
	 * 
	 * Ability to start / stop
	 * 	Individual / All
	 * Get status
	 * 	Individual / All
	 * Get results DONE
	 * 	Individual DONE
	 */

	private Logger log = new Logger( TaskHandler.class );

	private Map<Object, Callable<?>> tasks;
	private Map<Callable<?>, Future<?>> futures;
	private ExecutorService exe;

	public TaskHandler() {
		tasks = new HashMap<Object, Callable<?>>();
		futures = new HashMap<Callable<?>, Future<?>>();
		exe = Executors.newCachedThreadPool();
	}

	public TaskHandler( boolean singleThread ) {
		this();
		exe = Executors.newSingleThreadExecutor();
	}

	public void addTask( Object name, Callable<?> task ) { // Pass the variable to store the value in
		tasks.put( name, task );
		futures.put( task, exe.submit( task ) );
	}

	public void addTask( Object[] name, Callable<?> task ) {
		for( Object o : name ) {
			addTask( o, task );
		}
	}

	public Object getResult( Object name ) {
		if( isDone( name ) ) {
			try {
				return getFuture( name ).get();
			} catch (InterruptedException | ExecutionException e) {
				log.error("Failed to get result from future for task " + name.toString(), e);
			}
		}

		return null; // Not done yet
	}

	public Object forceGetResult( Object name ) { // Will hang until it gets the result
		try {
			return getFuture( name ).get();
		} catch (InterruptedException | ExecutionException e) {
			e.printStackTrace();
			log.error("Failed to get result from future for variable " + name.toString(), e);
		}
		return null;
	}

	public boolean isDone( Object name ) {
		if( inFutures( name ) ) {
			return getFuture( name ).isDone();
		}
		return false;
	}

	public boolean isRunning( Object name ) {
		if( futures.containsKey( tasks.get( name ) ) ) {
			if( !getFuture( name ).isDone() ) {
				return true;
			}
		}

		return false;
	}

	public void start( Object name ) {
		futures.put( tasks.get(name), exe.submit( tasks.get(name) ) );
	}

	public void stop( Object name ) {
		if( inFutures( name ) ) {
			getFuture( name ).cancel( true ); // Assume that whatever it is, force stop it
			futures.remove( tasks.get( name ) ); // Then remove it from running futures
		}
	}

	public void startAll() {
		// If there is a task that is not running, make it run
		for( Callable<?> f : tasks.values() ) {
			if( !futures.containsKey( f ) ) {
				futures.put( f, exe.submit( f ) );
			}
		}
	}

	public void stopAll() {
		for( Callable<?> c : futures.keySet() ) {
			futures.get( c ).cancel( true ); // Force stop the task, even if it is still running
			futures.remove( c ); // Remove it from running callables
		}

	}

	private boolean inTasks( Object name ) {
		if( tasks.containsKey( name ) ) {
			return true;
		}
		return false;
	}

	private boolean inFutures( Object name ) {
		if( inTasks( name ) && futures.containsKey( tasks.get( name ) ) ) {
			return true;
		}

		return false;
	}

	private Future<?> getFuture( Object name ){
		return futures.get( tasks.get( name ) );
	}
}
