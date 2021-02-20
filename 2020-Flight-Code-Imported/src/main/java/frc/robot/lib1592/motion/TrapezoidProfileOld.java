//TODO: If v0 > vmax and pTarget > 0, it jumps right down to the position without ramping
//		down from v0 to vmax

package frc.robot.lib1592.motion;

public class TrapezoidProfileOld {

	double velMax; 			//maximum speed of the profile
	double vPeak;			//peak speed during profile
	double accel;			//acceleration magnitude when changing velocity
	double pTarget;			//final target position
	double velInit;			//initial velocity
	double vFinal;
	
	double tAccelStop;		//time to stop accelerating
	double tDecelStart;		//time to begin decelerating
	double tAtTarget;		//time target is reached: profile finished
	
	double pAccelStop;		//distance covered during acceleration phase
	double pDecelStart;		//cumulative distance covered during acceleration and coast phases
	
	double sign = 1.0;		//Always compute positive profiles, so sign is -1.0 if pTarget < 0;
	
	public TrapezoidProfileOld(double pTarget,double v0,double vMax,double accel) {
		this(pTarget,v0,vMax,accel,0);
	}
	
	public TrapezoidProfileOld(double pTarget,double v0,double vMax,double accel,double vf)
	{
		//TODO: if v0 > vMax, profile immediately snaps down to vMax
		//Always compute positive trajectory, so flip if target is negative
		if (pTarget < 0) {
			sign = -1.0;
			pTarget = -pTarget;
			v0 = -v0;
		}
		
		//Assign Instance vars
		this.velMax = vMax;
		this.accel = accel;
		this.velInit = v0;
		this.pTarget = pTarget;
		this.vFinal = vf;
		
		//Peak velocity if executing triangular profile
		vPeak = Math.sqrt( (pTarget * accel) + (v0 * v0) / 2 + (vf*vf)/2);
		// time to stop accelerating. compute now to see if we're going to overshoot (tAccelStop < 0)
		tAccelStop = (vPeak-v0) / accel;
		double distAtConstVel = 0; //intermediate calc for trapezoidal profiles
		
		//Position covered if stopping immediately
		double pStopping = (v0*v0-vf*vf)/2/accel - MotionUtil.EPSILON;
		if (pStopping > pTarget) {
			System.out.println("Overshooting goal");
			//---Can't decelerate fast enough; have to overshoot and come back---
			//NOTE: setting vf = 0. Doesn't make sense to overshoot, come back, then speed back up
			vf = 0;
			//Distance overshooting the target is the dist required to stop minus pTarget
			double pOver = v0*v0/2/accel - pTarget;
			//peak velocity coming back to target
			double vPeakMag = Math.sqrt(pOver*accel);
			if (vPeakMag < vMax) {
				//-triangle-
				tAccelStop = tDecelStart = (v0 + vPeakMag) / accel;
				tAtTarget = tDecelStart + vPeakMag/accel;
				pAccelStop = pDecelStart = (v0*v0 - vPeakMag*vPeakMag)/2/accel;
			} else {
				//-trapezoid-
				vPeakMag = vMax;
				//time to stop accelerating (decelerating in this case)
				tAccelStop = (v0 + vPeakMag) / accel;
				pAccelStop = (v0*v0 - vPeakMag*vPeakMag)/2/accel;
				//distance covered at max velocity
				distAtConstVel = pTarget - pAccelStop + vPeakMag*vPeakMag/2/accel;
				//cumulative distance during accel and coast
				pDecelStart = pAccelStop + distAtConstVel;
				//time to start decelerating (accel back to target in this case)
				tDecelStart = tAccelStop - distAtConstVel/vMax;
				//time at target (assumes vf = 0)
				tAtTarget = tDecelStart + vMax/accel;
			}
			//set peak velocity
			vPeak = -vPeakMag;
			this.accel = -accel;
		}
		else if (vPeak <= vMax) {
			//---Triangular---
			//time to start decelerating
			tDecelStart = tAccelStop;
			//Distance covered while accelerating
			pAccelStop = pDecelStart = (vPeak*vPeak - v0*v0)/2/accel;
			//time at target
			tAtTarget = tDecelStart + (vPeak - vf)/accel;
		}
		else {
			//---Trapezoid---
			vPeak = vMax;
			//time to stop accelerating
			tAccelStop = (vPeak - v0) / accel;
			//distance covered while accelerating
			pAccelStop = (vPeak*vPeak - v0*v0)/2/accel;
			//distance covered while decelerating
			double distDuringDecel = (vPeak*vPeak - vf*vf)/2/accel;
			//distance covered at max velocity
			distAtConstVel = pTarget - pAccelStop - distDuringDecel;
			//cumulative distance during accel and coast
			pDecelStart = pAccelStop + distAtConstVel;
			//time to start decelerating
			tDecelStart = tAccelStop + distAtConstVel/vMax;
			//time at target
			tAtTarget = tDecelStart + (vPeak - vf)/accel;
		}
			

	} //constructor

	public TrajectoryPoint getTrajPoint(double t) {
		//Init to done case
		double p = pTarget*sign;
		double v = vFinal;
		double a = 0;
		double dt = 0;
		if (t >= tAtTarget) {
			//done
		}
		else if (t > tDecelStart) {
			//Decelerating
			dt = t - tDecelStart;
			a = sign * -accel;
			v = a*dt + sign*vPeak;
			p = v*dt + 0.5*(sign*vPeak - v)*dt + sign*pDecelStart;
		}
		else if (t > tAccelStop) {
			//Cruising at max velocity
			dt = t - tAccelStop;
			v = sign * vPeak;
			p = v*dt + sign*pAccelStop;
		}
		else {
			//accelerating
			a = sign * accel;
			v = a*t + sign*velInit;
			p = v*t + 0.5*(sign*velInit - v)*t;
		}
		return new TrajectoryPoint(t,p,v,a);
		
	}
	
	public double getTrajEndTime() {
		return tAtTarget;
	}
	
	//TODO: write method to generate full profile?  Input is dt.
} //class
