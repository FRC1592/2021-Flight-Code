package frc.robot.lib1592.drivers;

import edu.wpi.first.wpilibj.DigitalInput;

public class Photogate extends DigitalInput {
	public Photogate(int port)
	{
		super(port);
	}
	
	public boolean isBlocked()
	{
		return (!get());
	}
}