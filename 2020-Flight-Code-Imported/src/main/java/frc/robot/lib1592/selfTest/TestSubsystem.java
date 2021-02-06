package frc.robot.lib1592.selfTest;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

// Changed Command to CommandBase b/c can't extend interface.
// Affected call to start() in line 14 and interrupted() below.
public class TestSubsystem extends CommandBase {
	Testable subsystem;
	
	public TestSubsystem(Testable subsystem) {
		System.out.println("Test Cmd Instan");
		this.subsystem = subsystem;
		addRequirements((Subsystem)subsystem);
		// start();
	}
	
	@Override
	public void initialize() {
//		while(true) {
//			System.out.println("loop");
//		}
		subsystem.testSubsystem();
	}

	@Override
	public boolean isFinished() {
		System.out.println("Checking if finished");
		return false;
	}
	
	// @Override
	// protected void interrupted() {
	// 	System.out.println("I died");
	// }

}
