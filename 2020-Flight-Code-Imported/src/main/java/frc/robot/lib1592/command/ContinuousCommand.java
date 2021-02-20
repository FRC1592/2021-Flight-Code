package frc.robot.lib1592.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command will continuously execute.
 *
 * <p>Subclassing {@link ContinuousCommand} is shorthand for returning false from
 * {@link Command isFinished}.
 */
public class ContinuousCommand extends CommandBase {
	
	/**
	 * Creates a new {@link ContinuousCommand}. The name of this command will be set to its class name.
	 */
	public ContinuousCommand() {}

	/**
	 * Creates a new {@link ContinuousCommand} with the given name.
	 * 
	 * @param name  the name for this command
	 * @throws IllegalArgumentException if name is null
	 */
	public ContinuousCommand(String name) {
		super(); // Removed name due to change from 2019 to 2020 command framework.
	}

	@Override
	public boolean isFinished() {
		return false;
	}

}
