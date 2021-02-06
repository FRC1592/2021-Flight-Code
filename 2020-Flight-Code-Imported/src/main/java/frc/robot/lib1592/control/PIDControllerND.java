package frc.robot.lib1592.control;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.lib1592.control.InterfacePID.AbstractPID;
import frc.robot.lib1592.control.InterfacePID.ActionableTask;
import frc.robot.lib1592.control.InterfacePID.ChildPID;
import frc.robot.lib1592.control.InterfacePID.PIDOutputND;
import frc.robot.lib1592.control.InterfacePID.Tolerance;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * PID Controller Group
 */
public class PIDControllerND extends SendableBase implements AbstractPID {

	//======================//
	//      Properties      //
	//======================//
	
	public volatile Tolerance tolerance = null;					// Tolerance Target
	public volatile long period = BuilderPID.kDefaultPeriod;	// Loop Period (milliseconds)
	public volatile boolean enabled = false;					// Enabled PID On Startup
	
	private final List<PIDBase> pids;				// List Of PID Objects
	private ActionableTask m_task;					// Control Loop Task
	private java.util.Timer m_controlLoop;			// Control Loop Thread
	
	private volatile double m_result = 0;									// PID Result
	private volatile PIDOutput m_pidOutput = BuilderPID.emptySink;			// PID Output
	private volatile double[] m_results;									// PID Result ND
	private volatile PIDOutputND m_pidOutputND = (v) -> {};					// PID Output ND



	//========================//
	//      Constructors      //
	//========================//
	
	/**
	 * Builds a Group Of PIDs
	 * <p>
	 * This methods constructs multiple PID objects which operate on a
	 * single thread at the default period. The period setting in each
	 * builder object is ignored as the master thread period is used for
	 * an update rate. Therefore only group PIDs which should operate at
	 * the same rate. Additionally, the enabled setting in each builder is
	 * ignored as the enabled state of all of the child PID objects is maintained
	 * at the state of the PIDGroup. While you can set an output to the group,
	 * which will output the calculated group result, the individual outputs on
	 * each child are still active as is.
	 * <p>
	 * The PIDGroup tolerance is initialized to null (indicating never on target),
	 * but can be changed using {@link #setTolerance(Tolerance)}. Additionally, the
	 * PIDGroup is initialized as disabled, but can be enabled using 
	 * {@link #setEnabled(boolean)}. Finally, the PIDOutput and PIDOutputND are
	 * initialized to empty sinks, but can be changed using {@link #setOutput(PIDOutput)}
	 * and {@link #setOutput(PIDOutputND)}.
	 *
	 * @param builders  a list/array of PID builder objects
	 * @throws NullPointerException if {@code builders} is null or contains null elements
	 * @throws IllegalArgumentException if {@code builders} is empty
	 */
	public PIDControllerND(BuilderPID... builders) {
		this(BuilderPID.kDefaultPeriod, Arrays.asList(builders));
	}
	
	/**
	 * Builds a Group Of PIDs
	 * <p>
	 * This methods constructs multiple PID objects which operate on a
	 * single thread at the default period. The period setting in each
	 * builder object is ignored as the master thread period is used for
	 * an update rate. Therefore only group PIDs which should operate at
	 * the same rate. Additionally, the enabled setting in each builder is
	 * ignored as the enabled state of all of the child PID objects is maintained
	 * at the state of the PIDGroup. While you can set an output to the group,
	 * which will output the calculated group result, the individual outputs on
	 * each child are still active as is.
	 * <p>
	 * The PIDGroup tolerance is initialized to null (indicating never on target),
	 * but can be changed using {@link #setTolerance(Tolerance)}. Additionally, the
	 * PIDGroup is initialized as disabled, but can be enabled using 
	 * {@link #setEnabled(boolean)}. Finally, the PIDOutput and PIDOutputND are
	 * initialized to empty sinks, but can be changed using {@link #setOutput(PIDOutput)}
	 * and {@link #setOutput(PIDOutputND)}.
	 *
	 * @param builders  a list/array of PID builder objects
	 * @throws NullPointerException if {@code builders} is null or contains null elements
	 * @throws IllegalArgumentException if {@code builders} is empty
	 */
	public PIDControllerND(List<BuilderPID> builders) {
		this(BuilderPID.kDefaultPeriod, builders);
	}
	
	/**
	 * Builds a Group Of PIDs
	 * <p>
	 * This methods constructs multiple PID objects which operate on a
	 * single thread at the specified period. The period setting in each
	 * builder object is ignored as the master thread period is used for
	 * an update rate. Therefore only group PIDs which should operate at
	 * the same rate. Additionally, the enabled setting in each builder is
	 * ignored as the enabled state of all of the child PID objects is maintained
	 * at the state of the PIDGroup. While you can set an output to the group,
	 * which will output the calculated group result, the individual outputs on
	 * each child are still active as is.
	 * <p>
	 * The PIDGroup tolerance is initialized to null (indicating never on target),
	 * but can be changed using {@link #setTolerance(Tolerance)}. Additionally, the
	 * PIDGroup is initialized as disabled, but can be enabled using 
	 * {@link #setEnabled(boolean)}. Finally, the PIDOutput and PIDOutputND are
	 * initialized to empty sinks, but can be changed using {@link #setOutput(PIDOutput)}
	 * and {@link #setOutput(PIDOutputND)}.
	 *
	 * @param period  Loop Period (milliseconds)
	 * @param builders  a list/array of PID builder objects
	 * @throws NullPointerException if {@code builders} is null or contains null elements
	 * @throws IllegalArgumentException if {@code builders} is empty
	 */
	public PIDControllerND(long period, BuilderPID... builders) {
		this(period, Arrays.asList(builders));
	}
	
	/**
	 * Builds a Group Of PIDs
	 * <p>
	 * This methods constructs multiple PID objects which operate on a
	 * single thread at the specified period. The period setting in each
	 * builder object is ignored as the master thread period is used for
	 * an update rate. Therefore only group PIDs which should operate at
	 * the same rate. Additionally, the enabled setting in each builder is
	 * ignored as the enabled state of all of the child PID objects is maintained
	 * at the state of the PIDGroup. While you can set an output to the group,
	 * which will output the calculated group result, the individual outputs on
	 * each child are still active as is.
	 * <p>
	 * The PIDGroup tolerance is initialized to null (indicating never on target),
	 * but can be changed using {@link #setTolerance(Tolerance)}. Additionally, the
	 * PIDGroup is initialized as disabled, but can be enabled using 
	 * {@link #setEnabled(boolean)}. Finally, the PIDOutput and PIDOutputND are
	 * initialized to empty sinks, but can be changed using {@link #setOutput(PIDOutput)}
	 * and {@link #setOutput(PIDOutputND)}.
	 *
	 * @param period  Loop Period (milliseconds)
	 * @param builders  a list/array of PID builder objects
	 * @throws NullPointerException if {@code builders} is null or contains null elements
	 * @throws IllegalArgumentException if {@code builders} is empty
	 */
	public PIDControllerND(long period, List<BuilderPID> builders) {
		super(false);
		if (builders == null) {throw new NullPointerException();}
		if (builders.isEmpty()) {throw new IllegalArgumentException();}
		for (BuilderPID b : builders) {
			if (b == null) {throw new NullPointerException();}
		}
		this.period = period <= 0 ? BuilderPID.kDefaultPeriod : period;
		pids = new ArrayList<>(builders.size());
		for (BuilderPID b : builders) {
			b.period = this.period;
			b.enabled = false;
			pids.add(new PIDBase(b));
		}
		m_results = new double[builders.size()];
		m_controlLoop = new java.util.Timer();
		m_task = new ActionableTask(this);
		m_controlLoop.schedule(m_task, 0L, getPeriod());
	}



	//============================//
	//      Override Methods      //
	//============================//
	
	/**
	 * Method for aggregating the results/error from each child PID into
	 * a single result/error value. This method should be overridden if the
	 * standard methodology of using the L2-norm as the aggregate
	 * function does not work for the use case.
	 *
	 * @param results  an array of the child PID results/errors (consistent ordering with the builder order on creation)
	 * @return the aggregated result/error
	 */
	protected double aggregate(double[] results) {
		double out = 0;
		for (double v : results) {
			out += Math.pow(v, 2.0);
		}
		return Math.sqrt(out);
	}



	//===================//
	//      Methods      //
	//===================//
	
	// @Override public void free() {
	// 	for (PIDBase p : pids) {
	// 		p.free();
	// 	}
	// 	super.free();
	// }
	
	/**
	 * Disables the group and resets each child PID.
	 */
	public void reset() {
		synchronized (this) {
			setEnabled(false);
			for (PIDBase p : pids) { p.reset(); }
		}
	}
	
	/**
	 * Returns an independent list of the children PID objects.
	 * <p>
	 * This list is completely independent of the internally stored list
	 * so all list actions are permissible, but have no effect on the group.
	 * You can however use the returned items to adjust settings and acquire
	 * information about the performance of each individual PID.
	 *
	 * @return an independent list of the children PID objects.
	 */
	public List<ChildPID> getChildren() {
		List<ChildPID> out = new ArrayList<>(pids.size());
		for (PIDBase p : pids) { out.add(p); }
		return out;
	}
	
	/**
	 * Returns the number of child PID objects.
	 *
	 * @return the number of child PID objects.
	 */
	public int getDimensions() {
		return pids.size();
	}
	
	/**
	 * Returns the setpoints for each child PID object.
	 *
	 * @return the setpoint for each child PID object
	 */
	public double[] getSetpoint() {
		double[] out = new double[getDimensions()];
		int index = 0;
		for (ChildPID c : getChildren()) {
			out[index++] = c.getSetpoint();
		}
		return out;
	}
	
	/**
	 * Sets the setpoint to the given value.
	 *
	 * @param index  the pid index
	 * @param setpoint the new setpoint
	 */
	public void setSetpoint(int index, double setpoint) {
		getChildren().get(index).setSetpoint(setpoint);
	}

	/**
	 * Sets the setpoints to the given value.
	 *
	 * @param setpoints the new setpoints
	 */
	public void setSetpoints(double... setpoints) {
		int index = 0;
		for (ChildPID c : getChildren()) {
			if (index >= setpoints.length) {
				c.setSetpoint(0);
			} else {
				c.setSetpoint(setpoints[index++]);
			}
		}
	}

	@Override public Tolerance getTolerance() { return tolerance; }

	@Override public void setTolerance(Tolerance tolerance) { synchronized (this) { this.tolerance = tolerance; } }

	@Override public long getPeriod() { return period; }
	
	@Override public void setPeriod(long period) {
		if (period <= 0) {throw new IllegalArgumentException();}
		synchronized (this) {
			this.period = period;
			for (PIDBase p : pids) { p.setPeriod(this.period); }
			m_task.cancel();
			m_task = new ActionableTask(this);
			m_controlLoop.schedule(m_task, 0L, getPeriod());
		}
	}

	@Override public boolean isEnabled() { return enabled; }

	@Override public void setEnabled(boolean enabled) {
		synchronized (this) {
			for (PIDBase p : pids) { p.setEnabled(enabled); }
			this.enabled = enabled;
		}
	}
	
	@Override public PIDOutput getOutput() { return m_pidOutput; }

	@Override public void setOutput(PIDOutput output) {
		if (output == null) {throw new NullPointerException();}
		synchronized (this) { m_pidOutput = output; }
	}
	
	/** Returns the sink for the PID output (n-dimensional) */
	public PIDOutputND getOutputND() {return m_pidOutputND; }
	
	/**
	 * Sets the sink for the PID output (n-dimensional)
	 *
	 * @param output  the PID output sink (n-dimensional)
	 * @throws NullPointerException if {@code sink} is null
	 */
	public void setOutput(PIDOutputND output) {
		if (output == null) {throw new NullPointerException();}
		synchronized (this) { m_pidOutputND = output; }
	}

	@Override public void calculate() {
		int index = 0;
		double[] results = new double[pids.size()];
		synchronized (this) {
			for (PIDBase p : pids) {
				p.calculate();
				results[index++] = p.get();
			}
			m_results = results;
			m_pidOutputND.pidWrite(m_results);
			m_result = aggregate(results);
			m_pidOutput.pidWrite(m_result);
		}
	}

	@Override public double get() { return m_result; }
	
	@Override public double getError() {
		int index = 0;
		double[] errors = new double[pids.size()];
		synchronized (this) {
			for (PIDBase p : pids) {
				errors[index++] = p.getError();
			}
			return aggregate(errors);
		}
	}

	@Override public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("PIDGroup");
		builder.setSafeState(this::reset);
		builder.addBooleanProperty("enabled", this::isEnabled, this::setEnabled);
	}

}

