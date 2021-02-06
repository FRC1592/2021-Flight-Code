package frc.robot.lib1592.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 *
 */
public class AirPressureSensor {
    private final AnalogInput mAnalogInput;

    public AirPressureSensor(int analogInputNumber) {
        mAnalogInput = new AnalogInput(analogInputNumber);
    }
    
	public double getVoltage() {	
		return mAnalogInput.getVoltage();
	}
	
	/**
	 * Read the air pressure
	 * @return  pressure in PSI
	 */
	public double getPressure()	{
		//Measurement data (PSI,Volts): (100,4.2); (50,2.13); (20,1)
		return getVoltage() * 25 - 5;
	}
}
