package frc.robot.lib1592.control;

import frc.robot.lib1592.control.InterfacePID.ActionableTask;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * PID Controller
 */
public class PIDController extends PIDBase {

	//======================//
	//      Properties      //
	//======================//
	
	private ActionableTask m_task;				// Control Loop Task
	private java.util.Timer m_controlLoop;		// Control Loop Thread



	//========================//
	//      Constructors      //
	//========================//
	
	/**
	 * Create a full PID Controller object using the supplied builder.
	 * <p>
	 * If the builder is not valid (i.e {@link BuilderPID#isValid()} returns false), the
	 * builder is copied using {@link BuilderPID#copy()} and then made valid using
	 * {@link BuilderPID#makeValid()}.
	 *
	 * @param builder  the object on which to base the created PID Controller
	 * @throws NullPointerException if {@code builder} is null
	 */
	public PIDController(BuilderPID builder) {
		super(builder);
		m_controlLoop = new java.util.Timer();
		m_task = new ActionableTask(this);
		m_controlLoop.schedule(m_task, 0L, getPeriod());
	}



	//===================//
	//      Methods      //
	//===================//
	
	// @Override public void free() {
	// 	synchronized (this) {
	// 		m_controlLoop.cancel();
	// 		m_controlLoop = null;
	// 		super.free();
	// 	}
	// }
	
	@Override public void setPeriod(long period) {
		synchronized (this) {
			super.setPeriod(period);
			m_task.cancel();
			m_task = new ActionableTask(this);
			m_controlLoop.schedule(m_task, 0L, getPeriod());
		}
	}
	
	@Override public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.setSmartDashboardType("PIDController");
	}

}

