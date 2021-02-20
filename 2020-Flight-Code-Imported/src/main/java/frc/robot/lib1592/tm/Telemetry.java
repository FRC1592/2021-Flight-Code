package frc.robot.lib1592.tm;

import java.io.IOException;
import java.time.ZonedDateTime;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

import frc.robot.lib1592.threads.BackgroundThread;

/**
 * Telemetry Class
 * <p>
 * An instantiation of this class controls all of the telemetry handling. Users must register telemetry 
 * streams with this class and perform any other setup prior to calling the {@link #run} method. Once locked,
 * only a few high-level methods will remain accessible.
 */
public final class Telemetry {
	
	//===============================//
	//      Internal Components      //
	//===============================//
	
	private final Map<String, TelemetryStream> streamMap;
	
	private boolean locked;
	private boolean running;
	private boolean paused;
	private boolean closed;
	private BackgroundThread thread;
	private final int priority;
	
	
	
	//=======================//
	//      Constructor      //
	//=======================//
	
	/**
	 * Construct a new telemetry manager object.
	 */
	public Telemetry() {
		this(Thread.NORM_PRIORITY);
	}
	
	/**
	 * Construct a new telemetry manager object using the specified priority for the telemetry thread.
	 * 
	 * @param priority  the telemetry thread priority
	 */
	public Telemetry(int threadPriority) {
		streamMap = new HashMap<>();
		locked = false;
		paused = false;
		running = false;
		closed = false;
		priority = threadPriority;
	}
	
	
	
	//===========================//
	//      Public Methods       //
	//===========================//
	
	/**
	 * Returns whether the telemetry handler has been locked from editing. This is {@code true}
	 * any time after the first call to {@link #run()}.
	 */
	public boolean isLocked() {
		return locked;
	}
	
	/**
	 * Returns whether the telemetry handler is currently streaming telemetry. This is {@code true}
	 * after any valid call to {@link #run()}, but will be {@code false} after any valid
	 * call to {@link #pause()} or a call to {@link #close()}.
	 */
	public boolean isRunning() {
		return running;
	}
	
	/**
	 * Returns whether the telemetry handler is closed. This is {@code true}
	 * any time after the first call to {@link #close()}. This object is unusable once marked
	 * as closed.
	 */
	public boolean isClosed() {
		return closed;
	}
	
	/**
	 * Returns the unmodifiable telemetry stream map containing the active telemetry streams.
	 */
	public final Map<String, TelemetryStream> getUnmodifiableTelemetryStreamMap() {return Collections.unmodifiableMap(streamMap);}
	
	/**
	 * Adds the specified telemetry stream with the input name to the telemetry stream map, if the
	 * map does not already contain a telemetry stream by that name, otherwise does
	 * nothing. The telemetry stream associated with the input name is returned whether
	 * it is the specified stream or a previously created one.
	 * <p>
	 * This method returns null if attempted after {@link #isLocked()} begins to return {@code true}.
	 *
	 * @param name  the telemetry stream name
	 * @param stream  the telemetry stream
	 * @return the telemetry stream associated with the input name
	 */
	public final TelemetryStream addTelemetryStream(String name, TelemetryStream stream) {
		if (isLocked()) {return null;}
		if (name==null) {name = "";}
		TelemetryStream out = streamMap.get(name);
		if (out==null) {
			out = stream;
			streamMap.put(name, out);
		}
		return out;
	}
	
	/**
	 * Removes the telemetry stream with the specified input name, if any.
	 * <p>
	 * This method returns null if attempted after {@link #isLocked()} begins to return {@code true}.
	 *
	 * @param name  the background thread name to remove
	 * @return the inactive removed thread, if any, otherwise null
	 */
	public final TelemetryStream removeTelemetryStream(String name) {
		if (isLocked()) {return null;}
		return streamMap.remove(name);
	}
	
	/**
	 * Initializes the telemetry by locking/initializing all of the telemetry streams and 
	 * locking this object. Any subsequent calls to this when {@link #isLocked()} is true
	 * do nothing. 
	 * <p>
	 * Unlike the other telemetry methods, this method is synchronous and does
	 * not return until all of the streams have completed their 
	 * {@link TelemetryStream#initializeStream(ZonedDateTime)} method calls.
	 */
	public void init() {
		if (isLocked()) {return;}
		locked = true;
		thread = new BackgroundThread(priority);
		thread.open();
		ZonedDateTime dateTime = ZonedDateTime.now();
		for (TelemetryStream ts : streamMap.values()) {
			ts.lock();
			try {
				ts.initializeStream(dateTime);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
	
	/**
	 * Runs the telemetry. If {@link #isLocked()} is false, this first calls {@link #init()}. Any subsequent calls 
	 * to this method when {@link #isRunning()} is true do nothing. If {@link #isRunning()} is false and 
	 * {@link #isClosed()} is false, then a subsequent call to this method will resume data streaming.
	 */
	public void run() {
		if (isLocked()) {
			if (!isRunning() && !isClosed()) {
				if (paused) {
					for (TelemetryStream ts : streamMap.values()) {
						thread.run(() -> {try {
							ts.continueStream();
						} catch (IOException e) {
							e.printStackTrace();
						}});
					}
					paused = false;
				}
				running = true;
				for (Map.Entry<String, TelemetryStream> e : streamMap.entrySet()) {
					TelemetryStream ts = e.getValue();
					thread.startPeriodic(e.getKey(), ts.getPeriod(), () -> {try {
						ts.updateStream();
					} catch (IOException e1) {
						e1.printStackTrace();
					}});
				}
			}
		} else {
			init();
			running = true;
			for (Map.Entry<String, TelemetryStream> e : streamMap.entrySet()) {
				TelemetryStream ts = e.getValue();
				thread.startPeriodic(e.getKey(), ts.getPeriod(), () -> {try {
					ts.updateStream();
				} catch (IOException e1) {
					e1.printStackTrace();
				}});
			}
		}
	}
	
	/** Method for pausing all of the telemetry streams. This can occur only after
	 * the streams have been started using {@link #run()}, but before a call to {@link #close()}.
	 * If this is called when {@link #isRunning()} is {@code false}, nothing happens. Streams
	 * should strive to leave themselves in a valid state after pause such that abrupt exiting
	 * would still leave the stream in a valid state.
	 */
	public final void pause() {
		if (isLocked() && isRunning() && !isClosed()) {
			running = false;
			paused = true;
			thread.stopAllPeriodic();
			for (TelemetryStream ts : streamMap.values()) {
				thread.run(() -> {try {
					ts.pauseStream();
				} catch (IOException e) {
					e.printStackTrace();
				}});
			}
		}
	}
	
	/** Method for closing/finalizing all of the telemetry streams. This can occur only after
	 * the streams have been started using {@link #init()}
	 */
	public final void close() {
		if (isLocked()) {
			running = false;
			closed = true;
			thread.stopAllPeriodic();
			for (TelemetryStream ts : streamMap.values()) {
				thread.run(() -> {try {
					ts.finalizeStream();
				} catch (IOException e) {
					e.printStackTrace();
				}});
			}
			while (!streamMap.isEmpty()) {
				Iterator<Entry<String, TelemetryStream>> iter = streamMap.entrySet().iterator();
				while (iter.hasNext()) {
					if (!iter.next().getValue().isLocked()) {
						iter.remove();
					}
				}
			}
			thread.close();
		}
	}
	
}