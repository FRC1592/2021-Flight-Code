package frc.robot.lib1592.threads;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import frc.robot.lib1592.RobotUtility;

/**
 * Periodic Task Thread
 * <p>
 * The class implements a single background thread for which the user assigns one or more
 * periodic tasks at a given rate. This class provides simplified functionality for controlling
 * task behavior. Note that this class also provides the flexibility to change the periodic task
 * rate on the fly, though this can produce some slight lag behavior during the period change.
 * Additionally, if a period change is made while the tasks are paused, the tasks may (but are
 * not guaranteed) run at least once during the changeover before they resume their paused state.
 */
public final class PeriodicTaskThread {

	//===============================//
	//      Internal Components      //
	//===============================//

	private final BackgroundThread thread;
	private final HashMap<String, Runnable> actions;
	
	private long period;
	private boolean started;
	private boolean paused;
	private int count;



	//=======================//
	//      Constructor      //
	//=======================//
	
	/**
	 * Periodic Task thread constructor
	 * <p>
	 * This class runs at approximately 50 Hz and assigns the background thread to {@link Thread.NORM_PRIORITY}.
	 *
	 * @param period  the time in milliseconds between calling actions (i.e. 100 milliseconds is 10 Hz)
	 */
	public PeriodicTaskThread() {
		this(20, Thread.NORM_PRIORITY);
	}
	
	/**
	 * Periodic Task thread constructor
	 * <p>
	 * This class assigns the background thread to {@link Thread.NORM_PRIORITY}.
	 *
	 * @param period  the time in milliseconds between calling actions (i.e. 100 milliseconds is 10 Hz)
	 */
	public PeriodicTaskThread(long period) {
		this(period, Thread.NORM_PRIORITY);
	}

	/**
	 * Periodic Task thread constructor
	 *
	 * @param period  the time in milliseconds between calling actions (i.e. 100 milliseconds is 10 Hz)
	 * @param priority  the priority level of the thread
	 */
	public PeriodicTaskThread(long period, int priority) {
		thread = new BackgroundThread(priority);
		this.period = period;
		actions = new HashMap<>();
		started = false;
		paused = false;
	}



	//===========================//
	//      Public Methods       //
	//===========================//
	
	/**
	 * Adds the runnable to the task thread. If the thread is active, the runnable
	 * immediately begins scheduled execution. The runnable is named based on an
	 * internal counter.
	 *
	 * @param action  the runnable
	 * @return the resulting unique name for indexing usage if removing a runnable
	 */
	public final String addRunnable(Runnable action) {
		return addRunnable(Integer.toString(count++), action);
	}
	
	/**
	 * Adds the runnable to the task thread. If the thread is active, the runnable
	 * immediately begins scheduled execution.
	 *
	 * @param name  the runnable name
	 * @param action  the runnable
	 * @return the resulting unique name for indexing usage if removing a runnable
	 */
	public final String addRunnable(String name, Runnable action) {
		if (action==null) {throw new NullPointerException();}
		if (thread.isActive()) {
			name = thread.startPeriodic(name, this.period, action);
		} else {
			name = RobotUtility.ensureUnique(name, thread.getTasks());
		}
		actions.put(name, action);
		return name;
	}
	
	/**
	 * See {@link #addRunnable(Runnable)}.
	 *
	 * @param runnables  the runnables to add
	 * @return a list of the resulting unique names for indexing usage if removing a runnable
	 */
	public final List<String> addRunnables(Runnable... actions) {
		return addRunnables(Arrays.asList(actions));
	}
	
	/**
	 * See {@link #addRunnable(Runnable)}.
	 *
	 * @param runnables  the runnables to add
	 * @return a list of the resulting unique names for indexing usage if removing a runnable
	 */
	public final List<String> addRunnables(Collection<? extends Runnable> actions) {
		List<String> out = new ArrayList<>();
		for (Runnable r : actions) {
			out.add(addRunnable(r));
		}
		return out;
	}
	
	/**
	 * See {@link #addRunnable(String, Runnable)}.
	 * <p>
	 * Note that for this method call, the collection of the shortest length will dictate
	 * how many items are added to the task thread and therefore how many strings are returned.
	 *
	 * @param names  the runnable names
	 * @param runnables  the runnables to add
	 * @return a list of the resulting unique names for indexing usage if removing a runnable
	 */
	public final List<String> addRunnables(Collection<String> names, Collection<? extends Runnable> actions) {
		List<String> out = new ArrayList<>();
		Iterator<String> iterNames = names.iterator();
		Iterator<? extends Runnable> iterActions = actions.iterator();
		while (iterNames.hasNext() && iterActions.hasNext()) {
			out.add(addRunnable(iterNames.next(), iterActions.next()));
		}
		return out;
	}
	
	/**
	 * Removes the runnable from the task thread if found. If the thread is active,
	 * the runnable is also immediately stopped. If the named runnable is not found,
	 * no action is taken and this method returns false.
	 *
	 * @param name  the runnable to remove
	 * @return {@code true} if a runnable is removed, {@code false} otherwise
	 */
	public final boolean removeRunnable(String name) {
		Runnable removed = actions.remove(name);
		if (isActive()) {
			return thread.stopPeriodic(name);
		} else {
			return removed != null;
		}
	}
	
	/**
	 * See {@link #removeRunnable(String)}.
	 *
	 * @param name  the runnables to remove
	 * @return @return {@code true} if at least one runnable is removed, {@code false} otherwise
	 */
	public final boolean removeRunnables(String... name) {
		return removeRunnables(Arrays.asList(name));
	}
	
	/**
	 * See {@link #removeRunnable(String)}.
	 *
	 * @param name  the runnables to remove
	 * @return @return {@code true} if at least one runnable is removed, {@code false} otherwise
	 */
	public final boolean removeRunnables(Collection<String> name) {
		boolean out = false;
		for (String s : name) {
			out |= removeRunnable(s);
		}
		return out;
	}
	
	/**
	 * Opens the thread for activity. This method is a no-op if {@link #isActive()} already returns {@code true}.
	 */
	public final void open() {
		thread.open();
	}
	
	/**
	 * Sets the new priority level of the thread. If the thread is active, this method will
	 * momentarily stop all active tasks in order to perform a thread switch to a thread
	 * with the new priority before immediately resuming all tasks. Additionally, if a non-recurring
	 * task is queued but not running, it will be removed and never run.
	 *
	 * @param priority  the new priority level of the thread
	 */
	public final void setPriority(int priority) {
		thread.setPriority(priority);
	}
	
	/**
	 * Sets the time in milliseconds between subsequent calling actions. Note that this input is 
	 * fixed delay not fixed rate and will vary the exact separation based on the length of time 
	 * needed to run the runnables as well as other CPU considerations.
	 *
	 * @param period  the time in milliseconds between subsequent calling actions
	 * @throws IllegalArgumentException if {@code period <= 0}
	 */
	public void setPeriod(long period) {
		if (period<=0) {throw new IllegalArgumentException();}
		this.period = period;
		if (started) {
			if (paused) {
				stop();
				start();
				pause();
			} else {
				stop();
				start();
			}
		}
	}
	
	/**
	 * Returns the time in milliseconds between subsequent calling actions.
	 */
	public final long getPeriod() {
		return period;
	}
	
	/**
	 * Returns whether the background thread is active and therefore available for
	 * scheduling. This is {@code true} after {@link #open()} is called until
	 * {@link #close()} is called on this object, even if this cycle occurs repeatedly.
	 *
	 * @return whether the background thread is active and therefore available for scheduling
	 */
	public final boolean isActive() {
		return thread.isActive();
	}
	
	/**
	 * Returns whether the periodic tasks are running. This returns true after {@link #start()}
	 * has been called and remains true until either {@link #pause()}, {@link #stop()}, or
	 * {@link #close()} is called. If {@link #pause()} is called, this can return to true by
	 * a call to {@link #resume()}.
	 *
	 * @return whether the periodic tasks are running
	 */
	public final boolean isRunning() {
		return started && !paused;
	}
	
	/**
	 * Returns whether the periodic tasks are paused. This returns true after {@link #pause()}
	 * and will only become false by either {@link #resume()} or if the tasks are stopped using
	 * {@link #stop()} or {@link #close()}.
	 *
	 * @return Returns whether the periodic tasks are paused
	 */
	public final boolean isPaused() {
		return paused;
	}
	
	/**
	 * Starts the periodic tasks, if not already running, otherwise does nothing.
	 */
	public final void start() {
		if (!started) {
			started = true;
			for (Map.Entry<String, Runnable> e : actions.entrySet()) {
				thread.startPeriodic(e.getKey(), period, e.getValue());
			}
		}
	}
	
	/**
	 * Pauses the periodic tasks, if running, otherwise does nothing.
	 */
	public final void pause() {
		if (started && !paused) {
			paused = true;
			thread.pauseAllPeriodic();
		}
	}
	
	/**
	 * Resumes the periodic tasks, if paused, otherwise does nothing.
	 */
	public final void resume() {
		if (started && paused) {
			paused = false;
			thread.resumeAllPeriodic();
		}
	}
	
	/**
	 * Stops the periodic tasks, if running or paused, otherwise does nothing.
	 */
	public final void stop() {
		if (started) {
			started = false;
			paused = false;
			thread.stopAllPeriodic();
		}
	}
	
	/**
	 * Stops all of the periodic tasks, closes the background thread, and marks
	 * the thread as inactive. If the thread is already inactive, nothing is done.
	 */
	public final void close() {
		thread.close();
	}

}
