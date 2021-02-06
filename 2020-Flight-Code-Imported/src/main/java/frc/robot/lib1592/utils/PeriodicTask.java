package frc.robot.lib1592.utils;

import java.util.Timer;
import java.util.TimerTask;

public abstract class PeriodicTask {
	Timer timer;
	int timeMs;
	
	/**
	 * Create a new periodic task
	 * @param timeMs Time in milliseconds between runs
	 */
	public PeriodicTask(int timeMs) {
		timer = new Timer();
		timer.schedule(new Task(this), 0L, timeMs);
		this.timeMs = timeMs;
	}
	
	public int getPeriod() {
		return timeMs;
	}
	
	/**
	 * Free the PeriodicTask object
	 */
	public void free()
	{
		timer.cancel();
		synchronized(this)
		{
			timer = null;
		}
	}
	
	/**
	 * Override this method, this runs once every timeMs milliseconds
	 */
	public abstract void run();
	
	private class Task extends TimerTask {
		PeriodicTask task;
		
		public Task(PeriodicTask task) {
			this.task = task;
		}
		
		@Override
		public void run() {
			task.run();
		}
	}
}