package frc.robot.lib1592.drivers;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.lib1592.control.PIDConstantsCTRE;
import frc.robot.lib1592.drivers.TalonInterface.Action;
import frc.robot.lib1592.drivers.TalonInterface.ProtectedTalonSRX;

public class TalonSRX extends WPI_TalonSRX implements ProtectedTalonSRX, Action {
	
	//====================//
	//    Constructors    //
	//====================//
	
	/**
	 * Standard Talon Constructor
	 *
	 * @param id  motor controller id
	 */
	public TalonSRX(int id) {
		super(id);
	}
	
	/**
	 * Constructor With Initialized Constants
	 *
	 * @param id  motor controller id
	 * @param slot0  slot 0 PID constants
	 */
	public TalonSRX(int id, PIDConstantsCTRE slot0) {
		this(id);
		setPID(Slot.SLOT0, slot0);
	}

	/**
	 * Constructor With Dual Initialized Constants
	 *
	 * @param id  motor controller id
	 * @param slot0  slot 0 PID constants
	 * @param slot1  slot 1 PID constants
	 */
	public TalonSRX(int id, PIDConstantsCTRE slot0, PIDConstantsCTRE slot1) {
		this(id, slot0);
		setPID(Slot.SLOT1, slot1);
	}
	
	
	
	//===============//
	//    Getters    //
	//===============//
	
	/**
	 * PID Getter
	 *
	 * @param slot  the parameter slot
	 * @return a new PID constant object with all of the settings
	 */
	public PIDConstantsCTRE getPID(Slot slot) {
		return new PIDConstantsCTRE(getP(slot), getI(slot), getD(slot), getF(slot), getIZone(slot));
	}
	
	/**
	 * Proportional Getter
	 *
	 * @param slot  the parameter slot
	 * @return the proportional constant
	 */
	public double getP(Slot slot) {
		return configGetParameter(ParamEnum.eProfileParamSlot_P, slot.value, 0);
	}
	
	/**
	 * Integral Getter
	 *
	 * @param slot  the parameter slot
	 * @return the integral constant
	 */
	public double getI(Slot slot) {
		return configGetParameter(ParamEnum.eProfileParamSlot_I, slot.value, 0);
	}
	
	/**
	 * Derivative Getter
	 *
	 * @param slot  the parameter slot
	 * @return the derivative constant
	 */
	public double getD(Slot slot) {
		return configGetParameter(ParamEnum.eProfileParamSlot_D, slot.value, 0);
	}
	
	/**
	 * Feed-forward Getter
	 *
	 * @param slot  the parameter slot
	 * @return the feed-forward constant
	 */
	public double getF(Slot slot) {
		return configGetParameter(ParamEnum.eProfileParamSlot_F, slot.value, 0);
	}
	
	/**
	 * Integral Zone Getter
	 *
	 * @param slot  the parameter slot
	 * @return the integral zone constant
	 */
	public int getIZone(Slot slot) {
		return (int) configGetParameter(ParamEnum.eProfileParamSlot_IZone, slot.value, 0);
	}
	
	
	
	//===============//
	//    Setters    //
	//===============//
	
	/**
	 * PID Setter
	 *
	 * @param slot  the parameter slot
	 * @param consts  the pid constants
	 */
	public void setPID(Slot slot, PIDConstantsCTRE consts) {
		config_kP(slot.value, consts.kP, 0);
		config_kI(slot.value, consts.kI, 0);
		config_kD(slot.value, consts.kD, 0);
		config_kF(slot.value, consts.kF, 0);
		config_IntegralZone(slot.value, consts.kIz, 0);
	}
	
	/**
	 * Proportional Setter
	 *
	 * @param slot  the parameter slot
	 * @param p  the proportional constant
	 */
	public void setP(Slot slot, double p) {
		config_kP(slot.value, p, 0);
	}
	
	/**
	 * Integral Setter
	 *
	 * @param slot  the parameter slot
	 * @param i  the integral constant
	 */
	public void setI(Slot slot, double i) {
		config_kI(slot.value, i, 0);
	}
	
	/**
	 * Derivative Setter
	 *
	 * @param slot  the parameter slot
	 * @param d  the derivative constant
	 */
	public void setD(Slot slot, double d) {
		config_kD(slot.value, d, 0);
	}
	
	/**
	 * Feed-Forward Setter
	 *
	 * @param slot  the parameter slot
	 * @param f  the feed-forward constant
	 */
	public void setF(Slot slot, double f) {
		config_kF(slot.value, f, 0);
	}
	
	/**
	 * Integral Zone Setter
	 *
	 * @param slot  the parameter slot
	 * @param iZone  the integral zone constant
	 */
	public void setIZone(Slot slot, int iZone) {
		config_IntegralZone(slot.value, iZone, 0);
	}
	
	
	
	//===================//
	//    Inner Class    //
	//===================//
	
	public enum Slot {
		SLOT0(0),
		SLOT1(1);

		public final int value;

		private Slot(int v) {
			this.value = v;
		}
	}
	
}
