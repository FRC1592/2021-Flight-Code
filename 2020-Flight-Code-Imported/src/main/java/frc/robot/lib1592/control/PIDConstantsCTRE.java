/**

 */
package frc.robot.lib1592.control;

/**
 * Container to hold PID Constants for the TalonSRX.
 * TalonSRX Native Units:
 *   velocity = counts/100ms
 *   position = counts
 *   current = milliAmps
 *   Output = -1023 to 1023
 * Note: control loop runs at 1kHz (period = 1ms) but
 *   feedback signal may be sampled less frequently
 *   
 * @author dedyer1
 *
 */
public class PIDConstantsCTRE {
	//kF: Feed-forward 	[outputUnits/nativeUnits]
	//kP: Proportional	[outputUnits/nativeUnits]
	//kI: Integral		[outputUnits/nativeUnits/1ms]
	//kD: Derivative	[outputUnits/nativeUnits*1ms]
	public double kF, kP, kI, kD;
	//kIz: I-Zone 		[native units]
	public int kIz;
	
	/**
	 * Constructor for simple PID gains.
	 * Note: Izone (kIz) defaults to 75% of 1023/kP (for kP > 0,
	 * which is the error above which the Talon output is already
	 * saturated (therefore we don't want to accumulate any more error).
	 * 
	 * @param kP  proportional gain in [outputUnits/nativeUnits]
	 * @param kI  integral gain in [outputUnits/nativeUnits/1ms]
	 * @param kD  derivative gain in [outputUnits/nativeUnits*1ms]
	 */
	public PIDConstantsCTRE(double kP, double kI, double kD) {
		this(kP, kI, kD, 0, kP > 0 ? (int) (1023/kP*0.75) : 0);
	}
	
	/**
	 * Constructor for simple PIDF gains.
	 * Note: Izone (kIz) defaults to 75% of 1023/kP (for kP > 0,
	 * which is the error above which the Talon output is already
	 * saturated (therefore we don't want to accumulate any more error).
	 * 
	 * @param kP  proportional gain in [outputUnits/nativeUnits]
	 * @param kI  integral gain in [outputUnits/nativeUnits/1ms]
	 * @param kD  derivative gain in [outputUnits/nativeUnits*1ms]
	 * @param kF  feed forward gain in [outputUnits/nativeUnits]
	 */
	public PIDConstantsCTRE(double kP, double kI, double kD, double kF) {
		this(kP, kI, kD, kF, kP > 0 ? (int) (1023/kP*0.75) : 0);
	}

	/**
	 * Constructor for simple PIDF gains with integral zone.  If the 
	 * error is outside the Izone, the accumulated error is reset so
	 * the integral gain has no effect.
	 * Note: Izone (kIz) should be <= 1023/kP, which is the error
	 * above which the Talon output is already saturated (therefore
	 * we don't want to accumulate any more error).
	 * 
	 * @param kP  proportional gain in [outputUnits/nativeUnits]
	 * @param kI  integral gain in [outputUnits/nativeUnits/1ms]
	 * @param kD  derivative gain in [outputUnits/nativeUnits*1ms]
	 * @param kF  feed forward gain in [outputUnits/nativeUnits]
	 * @param kIz integral zone in [nativeUnits]
	 */
	public PIDConstantsCTRE(double kP, double kI, double kD, double kF, int kIz) {
		this.kF = kF;
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kIz = kIz;
		if (kP > 0 && kIz > 1023/kP) {
			System.err.println("Integral zone is set high and could create wind-up problems ");
		} else if (kI > 0 && kIz == 0){
			System.err.println("Integarl zone is set to zero and not protecting against wind-up");
		}
	}
	
	/**
	 * Helper method to convert encoder speed to a feed-forward gain
	 * @param RPM_enc: RPM measured at encoder
	 * @return Kf: TalonSRX feed-forward gain
	 */
	public static final double RPM_enc2Kf(final double RPM_enc) {
		//Encoder is 4096 pulses per rev; Talon time unit is 100ms
		final double RPM2COUNTS_PER_100MS = 4096.0/10.0/60.0;
		//maximum talon output is integer 1023
		final double maxOut = 1023.0;
		//Feed forward gain: kF
		return maxOut / (RPM_enc * RPM2COUNTS_PER_100MS);
	}
	
//	/**
//	 * Apply the PID Parameters to the specified CANTALON motor controller
//	 * @param motor
//	 */
//	public void applyConstants(TalonSRX motor) {
//		motor.setPID(kP, kI, kD);
//		motor.setF(kF);
//		motor.setIZone(kIz);
//		
//	}
}
