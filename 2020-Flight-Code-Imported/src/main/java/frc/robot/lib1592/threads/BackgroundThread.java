package frc.robot.lib1592.threads;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.TimerTask;

import frc.robot.lib1592.RobotUtility;

/**
 * Background Thread Class
 * <p>
 * An instantiation of this class initializes a java.util.Timer object, which itself
 * is a facility for scheduling and controlling tasks on a background thread. The thread
 * itself is instantiated by the java.util.Timer object.
 * <p>
 * This class provides a more convenient API for setting up and scheduling runnable tasks
 * as well as for closing the background thread.
 */
public final class BackgroundThread {

	//===============================//
	//      Internal Components      //
	//===============================//

	private java.util.Timer timer;
	private int threadLevel;
	private final Set<String> keys;
	private final Map<String, InternalTask> tasks;
	private final Map<String, TimedRunnable> pausedTasks;
	private boolean active;



	//=======================//
	//      Constructor      //
	//=======================//

	/**
	 * Standard thread constructor
	 *
	 * @param priority  the priority level of the thread
	 */
	public BackgroundThread(int priority) {
		threadLevel = priority;
		keys = new HashSet<>();
		tasks = new HashMap<>();
		pausedTasks = new HashMap<>();
	}



	//===========================//
	//      Public Methods       //
	//===========================//

	/**
	 * Opens the thread for activity. This method is a no-op if {@link #isActive()} already returns {@code true}.
	 */
	public final void open() {
		if (isActive()) {return;}
		Thread v = new Thread(() -> {
			timer = new java.util.Timer();
		});
		v.setPriority(threadLevel);
		v.start();
		try {
			v.join();
		} catch (InterruptedException e) {
			throw new IllegalArgumentException(e);
		}
		active = true;
	}

	/**
	 * Returns whether the background thread is active and therefore available for
	 * scheduling. This is {@code true} after {@link #open()} is called until
	 * {@link #close()} is called on this object, even if this cycle occurs repeatedly.
	 *
	 * @return whether the background thread is active and therefore available for scheduling
	 */
	public final boolean isActive() {
		return active;
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
		if (priority == threadLevel) {return;}
		if (isActive()) {
			Map<String, InternalTask> saveTasks = new HashMap<>(tasks);
			Map<String, TimedRunnable> savePausedTasks = new HashMap<>(pausedTasks);
			close();
			threadLevel = priority;
			open();
			for (Map.Entry<String, InternalTask> e : saveTasks.entrySet()) {
				startPeriodic(e.getKey(), e.getValue().period, e.getValue().action);
			}
			for (Map.Entry<String, TimedRunnable> e : savePausedTasks.entrySet()) {
				startPeriodic(e.getKey(), e.getValue().period, e.getValue().action);
				pausePeriodic(e.getKey());
			}
		} else {
			threadLevel = priority;
		}
	}

	/**
	 * Immediately runs the input action method on the background thread.
	 * <p>
	 * If {@link #isActive()} is {@code false}, this method calls {@link #open()} before
	 * executing the runnable.
	 *
	 * @param action  the runnable
	 */
	public final void run(Runnable action) {
		open();
		timer.schedule(new InternalTask(action, 0), 0);
	}

	/**
	 * Schedules the input action method on the background thread at the specified 
	 * period and registers that task with the specified input name. If the input 
	 * name already exists, a modified one is generated to ensure uniqueness. The 
	 * actually assigned name, modified as necessary for uniqueness, is returned by 
	 * this method.
	 * <p>
	 * If {@link #isActive()} is {@code false}, this method calls {@link #open()} before
	 * submitting the runnable to the schedule.
	 *
	 * @param name  the requested task name
	 * @param period  the time in milliseconds between method calls (fixed-delay)
	 * @param action  the runnable
	 * @return the name actually stored as reference to this periodic task
	 */
	public final String startPeriodic(String name, long period, Runnable action) {
		name = RobotUtility.ensureUnique(name, keys);
		InternalTask task = new InternalTask(action, period);
		keys.add(name);
		tasks.put(name, task);
		open();
		timer.schedule(task, 0, task.period);
		return name;
	}

	/**
	 * Stops the periodic task with the given name from continuing to execute as
	 * scheduled on the background thread. This method only succeeds if the input
	 * name matches a currently running named task, otherwise no changes are made.
	 * <p>
	 * If {@link #isActive()} is {@code false}, this method is a no-op and returns false.
	 *
	 * @param name  the reference name to a periodic task
	 * @return {@code true} if the name referred to a valid task which was stopped
	 * 			successfully, {@code false} otherwise.
	 */
	public final boolean pausePeriodic(String name) {
		if (!isActive()) {return false;}
		InternalTask task = tasks.get(name);
		if (task != null) {
			boolean out = task.cancel();
			if (out) {
				pausedTasks.put(name, new TimedRunnable(task.action, task.period));
			}
			return out;
		}
		return false;
	}

	/**
	 * Pauses all of the periodic tasks, if any, from continuing to execute as
	 * scheduled on the background thread.
	 * <p>
	 * If {@link #isActive()} is {@code false}, this method is a no-op.
	 */
	public final void pauseAllPeriodic() {
		if (!isActive()) {return;}
		for (Map.Entry<String, InternalTask> e : tasks.entrySet()) {
			e.getValue().cancel();
			pausedTasks.put(e.getKey(), new TimedRunnable(e.getValue().action, e.getValue().period));
		}
	}

	/**
	 * Resumes the periodic task with the given name, if it exists and is paused.
	 * <p>
	 * If {@link #isActive()} is {@code false}, this method is a no-op and returns false.
	 *
	 * @param name  the reference name to a periodic task
	 * @return {@code true} if the name referred to a valid task which was paused
	 * 			and is resumed successfully, {@code false} otherwise.
	 */
	public final boolean resumePeriodic(String name) {
		if (!isActive()) {return false;}
		TimedRunnable action = pausedTasks.remove(name);
		if (action != null) {
			InternalTask task = new InternalTask(action.action, action.period);
			tasks.put(name, task);
			timer.schedule(task, 0, action.period);
			return true;
		}
		return false;
	}

	/**
	 * Resumes all of the paused periodic tasks, if any.
	 * <p>
	 * If {@link #isActive()} is {@code false}, this method is a no-op and returns false.
	 * 
	 * @return {@code true} if any tasks are resumed successfully, {@code false} otherwise.
	 */
	public final boolean resumeAllPeriodic() {
		if (!isActive()) {return false;}
		boolean out = false;
		for (Map.Entry<String, TimedRunnable> e : pausedTasks.entrySet()) {
			InternalTask task = new InternalTask(e.getValue().action, e.getValue().period);
			tasks.put(e.getKey(), task);
			timer.schedule(task, 0, e.getValue().period);
			out = true;
		}
		pausedTasks.clear();
		return out;
	}

	/**
	 * Stops the periodic task with the given name from continuing to execute as
	 * scheduled on the background thread and removes them from the task list. 
	 * This method only succeeds if the input name matches a currently running 
	 * named task, otherwise no changes are made.
	 * <p>
	 * If {@link #isActive()} is {@code false}, this method is a no-op and returns false.
	 *
	 * @param name  the reference name to a periodic task
	 * @return {@code true} if the name referred to a valid task which was stopped
	 * 			successfully, {@code false} otherwise.
	 */
	public final boolean stopPeriodic(String name) {
		if (!isActive()) {return false;}
		InternalTask task = tasks.remove(name);
		if (task != null) {
			pausedTasks.remove(name);
			return task.cancel();
		}
		keys.remove(name);
		return false;
	}

	/**
	 * Stops all of the periodic tasks, if any, from continuing to execute as
	 * scheduled on the background thread and removes them from the task list.
	 * <p>
	 * If {@link #isActive()} is {@code false}, this method is a no-op.
	 */
	public final void stopAllPeriodic() {
		if (!isActive()) {return;}
		for (TimerTask task : tasks.values()) {
			task.cancel();
		}
		keys.clear();
		tasks.clear();
		pausedTasks.clear();
	}

	/**
	 * Returns an unmodifiable set of the names of the currently active periodic tasks, if any.
	 */
	public final Set<String> getActivePeriodic() {
		Set<String> out = new HashSet<>(tasks.keySet());
		out.removeAll(pausedTasks.keySet());
		return Collections.unmodifiableSet(out);
	}

	/**
	 * Returns an unmodifiable set of the names of the currently paused periodic tasks, if any.
	 */
	public final Set<String> getPausedPeriodic() {
		return Collections.unmodifiableSet(pausedTasks.keySet());
	}

	/**
	 * Returns an unmodifiable set of the names of all of the tasks (active or paused), if any.
	 */
	public final Set<String> getTasks() {
		return Collections.unmodifiableSet(keys);
	}

	/**
	 * Stops all of the periodic tasks, closes the background thread, and marks
	 * the thread as inactive. Additionally, if a non-recurring task is queued 
	 * but not running, it will be removed and never run. If the thread is already 
	 * inactive, nothing is done.
	 */
	public final void close() {
		if (!isActive()) {return;}
		stopAllPeriodic();
		timer.cancel();
		active = false;
	}



	//================================//
	//      Private Final Class       //
	//================================//

	/** Internal Task Implementation Wrapper */
	private final class InternalTask extends TimerTask {
		private final Runnable action;
		private final long period;
		InternalTask(Runnable action, long period) {
			if (action==null) {throw new NullPointerException();}
			this.action = action;
			this.period = period;
		}
		@Override public void run() {
			action.run();
		}
	}

	/** Timed Runnable Wrapper */
	private final class TimedRunnable {
		private final Runnable action;
		private final long period;
		TimedRunnable(Runnable action, long period) {
			this.action = action;
			this.period = period;
		}
	}

}