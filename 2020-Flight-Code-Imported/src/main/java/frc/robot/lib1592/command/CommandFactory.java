package frc.robot.lib1592.command;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Static Class For Building Basic Commands
 * <p>
 * This builder contains all of the basic methods for generating commands in an in-line manner
 * with most of the basic options available for construction. Advanced commanding, maintenance of
 * state in a command, or using advanced concepts like preventing interruption should be handled
 * by an individual class extended from {@link Command} or, more likely, one of its many already
 * extended classes.
 */
public class CommandFactory {
	
	/** Static Class */
	private CommandFactory() {}
	
	
	
	//========================//
	//   Instant Variations   //
	//========================//
	
	/**
	 * Generates a named command which runs the input action once.
	 * <p>
	 * The input action can be null, in which case nothing is run, but no
	 * exception is thrown.
	 *
	 * @param name  the command name
	 * @param action  the command action
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runOnce(String name, Runnable action) {
		return runOnce(name, action, null, null, (SubsystemBase[]) null);
	}
	
	/**
	 * Generates a named command which runs the input action once and requires the
	 * specified subsystem(s).
	 * <p>
	 * The input action can be null, in which case nothing is run, but no
	 * exception is thrown. The subsystem array can be null or any of its elements
	 * can be null. Each null item is simply ignored.
	 *
	 * @param name  the command name
	 * @param action  the command action
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runOnce(String name, Runnable action, SubsystemBase... required) {
		return runOnce(name, action, null, null, required);
	}
	
	/**
	 * Generates a named command which runs the input action once and requires the
	 * specified subsystem(s). The command also runs the completion method once either
	 * after the successful running of the input action or if the command is interrupted 
	 * prior to the action running.
	 * <p>
	 * The input action can be null, in which case nothing is run, but no
	 * exception is thrown. The subsystem array can be null or any of its elements
	 * can be null. Each null item is simply ignored. The completion action can be
	 * null, in which case no completion action is performed.
	 *
	 * @param name  the command name
	 * @param action  the command action
	 * @param completion  the completion action (for standard and interrupted completion)
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runOnce(String name, Runnable action, Runnable completion, SubsystemBase... required) {
		return runOnce(name, action, completion, completion, required);
	}
	
	/**
	 * Generates a named command which runs the input action once and requires the
	 * specified subsystem(s). The command also runs the completion method after the 
	 * successful running of the input action or the interruption method if the command 
	 * is interrupted prior to the action running.
	 * <p>
	 * The input action can be null, in which case nothing is executed, but no
	 * exception is thrown. The subsystem array can be null or any of its elements
	 * can be null. Each null item is simply ignored. The completion action can be
	 * null, in which case no completion action is performed. Additionally, the
	 * interruption action can be null, in which case no interrupted action is performed.
	 *
	 * @param name  the command name
	 * @param action  the command action
	 * @param completion  the standard completion action
	 * @param interruption  the interrupted completion action
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runOnce(String name, Runnable action, Runnable completion, Runnable interruption, SubsystemBase... required) {
		return new InternalConstantCommand(true, name, action, null, completion, interruption, required);
	}
	
	
	
	//===========================//
	//   Continuous Variations   //
	//===========================//
	
	/**
	 * Generates a named command which runs the input action continuously.
	 * <p>
	 * The input action can be null, in which case nothing is run at
	 * each execute call, but no exception is thrown.
	 *
	 * @param name  the command name
	 * @param action  the command action
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runContinuous(String name, Runnable action) {
		return runContinuous(name, action, null, (SubsystemBase[]) null);
	}
	
	/**
	 * Generates a named command which runs the input action continuously and requires the
	 * specified subsystem(s).
	 * <p>
	 * The input action can be null, in which case nothing is run at
	 * each execute call, but no exception is thrown. The subsystem array can be 
	 * null or any of its elements can be null. Each null item is simply ignored.
	 *
	 * @param name  the command name
	 * @param action  the command action
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runContinuous(String name, Runnable action, SubsystemBase... required) {
		return runContinuous(name, action, null, required);
	}
	
	/**
	 * Generates a named command which runs the input action continuously and requires the
	 * specified subsystem(s). The command also runs the input initialization action once
	 * prior to beginning the calls to the input action.
	 * <p>
	 * The input action can be null, in which case nothing is run at
	 * each execute call, but no exception is thrown. The subsystem array can be 
	 * null or any of its elements can be null. Each null item is simply ignored.
	 *
	 * @param name  the command name
	 * @param action  the command action
	 * @param initialization  the command initialization
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runContinuous(String name, Runnable action, Runnable initialization, SubsystemBase... required) {
		return runContinuous(name, action, initialization, null, required);
	}
	
	/**
	 * Generates a named command which runs the input action continuously and requires the
	 * specified subsystem(s). The command also runs the input initialization action once
	 * prior to beginning the calls to the input action. Additionally, it runs the interruption 
	 * method if the command is interrupted. There is no completion method provided as, due to 
	 * the continuous nature, this command will never complete normally.
	 * <p>
	 * The input action can be null, in which case nothing is run at
	 * each execute call, but no exception is thrown. The subsystem array can be 
	 * null or any of its elements can be null. Each null item is simply ignored.
	 * The interruption action can be null, in which case no interrupted action is 
	 * performed.
	 *
	 * @param name  the command name
	 * @param action  the command action
	 * @param initialization  the command initialization
	 * @param interruption  the interrupted completion action
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runContinuous(String name, Runnable action, Runnable initialization, Runnable interruption, SubsystemBase... required) {
		return new InternalConstantCommand(false, name, action, null, null, interruption, required);
	}
	
	
	
	//======================//
	//   Timed Variations   //
	//======================//
	
	/**
	 * Generates a named timed command which runs the input action until the timeout
	 * expires.
	 * <p>
	 * The input action can be null, in which case nothing is executed, but no
	 * exception is thrown.
	 *
	 * @param name  the command name
	 * @param timeout  the command timeout
	 * @param action  the command action
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runTimed(String name, double timeout, Runnable action) {
		return runTimed(name, timeout, action, null, null, (SubsystemBase[]) null);
	}
	
	/**
	 * Generates a named timed command which runs the input action until the timeout
	 * expires and requires the specified subsystem(s).
	 * <p>
	 * The input action can be null, in which case nothing is executed, but no
	 * exception is thrown. The subsystem array can be null or any of its elements
	 * can be null. Each null item is simply ignored.
	 *
	 * @param name  the command name
	 * @param timeout  the command timeout
	 * @param action  the command action
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runTimed(String name, double timeout, Runnable action, SubsystemBase... required) {
		return runTimed(name, timeout, action, null, null, required);
	}
	
	/**
	 * Generates a named timed command which runs the input action until the timeout
	 * expires and requires the specified subsystem(s). The command also runs the input 
	 * initialization action once prior to beginning the calls to the input action.
	 * <p>
	 * The input action can be null, in which case nothing is executed, but no
	 * exception is thrown. The subsystem array can be null or any of its elements
	 * can be null. Each null item is simply ignored.
	 *
	 * @param name  the command name
	 * @param timeout  the command timeout
	 * @param action  the command action
	 * @param initialization  the command initialization
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runTimed(String name, double timeout, Runnable action, Runnable initialization, SubsystemBase... required) {
		return runTimed(name, timeout, action, null, null, null, required);
	}
	
	/**
	 * Generates a named timed command which runs the input action until the timeout
	 * expires and requires the specified subsystem(s). The command also runs the input 
	 * initialization action once prior to beginning the calls to the input action. Additionally,
	 * it runs the completion action either after the successful running of the command or if 
	 * the command is interrupted prior to the timed completion.
	 * <p>
	 * The input action can be null, in which case nothing is executed, but no
	 * exception is thrown. The subsystem array can be null or any of its elements
	 * can be null. Each null item is simply ignored. The completion action can be
	 * null, in which case no completion action is performed.
	 *
	 * @param name  the command name
	 * @param timeout  the command timeout
	 * @param action  the command action
	 * @param initialization  the command initialization
	 * @param completion  the completion action (for standard and interrupted completion)
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runTimed(String name, double timeout, Runnable action, Runnable initialization, Runnable completion, SubsystemBase... required) {
		return runTimed(name, timeout, action, null, completion, completion, required);
	}
	
	/**
	 * Generates a named timed command which runs the input action until the timeout
	 * expires and requires the specified subsystem(s). The command also runs the input 
	 * initialization action once prior to beginning the calls to the input action. Additionally,
	 * it runs the completion method after the successful running of the command or the 
	 * interruption method if the command is interrupted prior to the timed completion.
	 * <p>
	 * The input action can be null, in which case nothing is executed, but no
	 * exception is thrown. The subsystem array can be null or any of its elements
	 * can be null. Each null item is simply ignored. The completion action can be
	 * null, in which case no completion action is performed. Additionally, the
	 * interruption action can be null, in which case no interrupted action is performed.
	 *
	 * @param name  the command name
	 * @param timeout  the command timeout
	 * @param action  the command action
	 * @param initialization  the command initialization
	 * @param completion  the standard completion action
	 * @param interruption  the interrupted completion action
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name is null
	 */
	public static CommandBase runTimed(String name, double timeout, Runnable action, Runnable initialization, Runnable completion, Runnable interruption, SubsystemBase... required) {
		return new InternalTimedCommand(name, timeout, action, initialization, completion, interruption, required);
	}

	
	
	//============================//
	//   Conditional Variations   //
	//============================//
	
	/**
	 * Generates a named conditional command which evaluates the input condition upon 
	 * initialization and, if true, starts the input command.
	 * <p>
	 * The input whenTrue command can be null, in which case, no command is started 
	 * if that branch is chosen by the conditional evaluation. The subsystem array 
	 * can be null or any of its elements can be null. Each null item is simply 
	 * ignored.
	 *
	 * @param name  the command name
	 * @param condition  the command condition
	 * @param whenTrue  the command to start if the condition evaluates true
	 * @return the generated command
	 * @throws IllegalArgumentException if name or condition is null
	 */
	public static CommandBase runConditional(String name, BooleanSupplier condition, CommandBase whenTrue) {
		return runConditional(name, condition, whenTrue, null, (SubsystemBase[]) null);
	}
	
	/**
	 * Generates a named conditional command which evaluates the input condition upon 
	 * initialization and, if true, starts the input command. The command also requires 
	 * the specified subsystem(s).
	 * <p>
	 * The input whenTrue command can be null, in which case, no command is started 
	 * if that branch is chosen by the conditional evaluation. The subsystem array 
	 * can be null or any of its elements can be null. Each null item is simply 
	 * ignored.
	 *
	 * @param name  the command name
	 * @param condition  the command condition
	 * @param whenTrue  the command to start if the condition evaluates true
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name or condition is null
	 */
	public static CommandBase runConditional(String name, BooleanSupplier condition, CommandBase whenTrue, SubsystemBase... required) {
		return runConditional(name, condition, whenTrue, null, required);
	}
	
	/**
	 * Generates a named conditional command which evaluates the input condition upon 
	 * initialization and starts the appropriate response command, if available. The 
	 * command also requires the specified subsystem(s).
	 * <p>
	 * The input whenTrue and whenFalse commands can be null, in which case, no command
	 * is started if that branch is chosen by the conditional evaluation. The subsystem 
	 * array can be null or any of its elements can be null. Each null item is simply 
	 * ignored.
	 *
	 * @param name  the command name
	 * @param condition  the command condition
	 * @param whenTrue  the command to start if the condition evaluates true
	 * @param whenFalse  the command to start if the condition evaluates false
	 * @param required  the required subsystem(s)
	 * @return the generated command
	 * @throws IllegalArgumentException if name or condition is null
	 */
	public static CommandBase runConditional(String name, BooleanSupplier condition, CommandBase whenTrue, CommandBase whenFalse, SubsystemBase... required) {
		return new InternalConditionalCommand(name, condition, whenTrue, whenFalse, required);
	}

	
	
	//======================//
	//   Other Variations   //
	//======================//
	
	/**
	 * Generates a command which prints the specified message to System.out
	 *
	 * @param message  the message to print
	 * @return the generated command
	 */
	public static CommandBase print(String message) {
		return new PrintCommand(message == null ? "" : message);
	}
	
	/**
	 * Generates a command which waits for the specified amount of time before finishing
	 *
	 * @param timeout  the time to wait (seconds)
	 * @return the generated command
	 */
	public static CommandBase wait(double timeout) {
		return new WaitCommand(timeout);
	}
	
	/**
	 * Generates a command which waits until the specified match time
	 * ({@link edu.wpi.first.wpilibj.Timer#getMatchTime() Timer.getMatchTime()})
	 *
	 * @param time  the match time to wait till
	 * @return the generated command
	 */
	public static CommandBase waitUntil(double time) {
		return new WaitUntilCommand(time);
	}
	
	// Removed for 2020.
	// /**
	//  * Generates a command which waits for all active children in a command
	//  * group to complete. This method completes immediately if outside of a
	//  * command group.
	//  *
	//  * @return the generated command
	//  * @see WaitForChildren
	//  */
	// public static CommandBase waitForChildren() {
	// 	return new WaitForChildren();
	// }

	
	
	//======================//
	//   Group Variations   //
	//======================//
	
	/**
	 * Generates a command group made up of each of the input commands
	 * occurring in parallel.
	 *
	 * @param commands  the commands to parallelize
	 * @return the generated command group
	 */
	public static ParallelCommandGroup parallel(CommandBase... commands) {
		ParallelCommandGroup out = new ParallelCommandGroup();
		return parallel(out, commands);
	}
	
	/**
	 * Generates a named command group made up of each of the input commands
	 * occurring in parallel.
	 *
	 * @param name  the command group name
	 * @param commands  the commands to parallelize
	 * @return the generated command group
	 */
	public static ParallelCommandGroup parallel(String name, CommandBase... commands) {
		ParallelCommandGroup out = new ParallelCommandGroup(); // Removed name for 2020.
		return parallel(out, commands);
	}
	
	/**
	 * Generates a command group made up of each of the input commands
	 * occurring in sequential order (as input).
	 *
	 * @param commands  the commands to sequence
	 * @return the generated command group
	 */
	public static SequentialCommandGroup sequential(CommandBase... commands) {
		SequentialCommandGroup out = new SequentialCommandGroup();
		return sequential(out, commands);
	}
	
	/**
	 * Generates a named command group made up of each of the input commands
	 * occurring in sequential order (as input).
	 *
	 * @param name  the command group name
	 * @param commands  the commands to sequence
	 * @return the generated command group
	 */
	public static SequentialCommandGroup sequential(String name, CommandBase... commands) {
		SequentialCommandGroup out = new SequentialCommandGroup(); // Removed name for 2019.
		return sequential(out, commands);
	}
	
	
	
	//=============================//
	//   Internal Implementation   //
	//=============================//
	
	/**
	 * Internal Command Extension
	 * <p>
	 * This class encapsulates holding runnable objects for usage during the execute(), initialize(), 
	 * end(), and interrupted() method calls. Additionally this class is built to use an input boolean
	 * to either emulate an instant command or a continuous command.
	 */
	private static class InternalConstantCommand extends CommandBase {
		private final boolean _finished;
		private final Runnable _action;
		private final Runnable _initialization;
		private final Runnable _completion;
		private final Runnable _interruption;
		
		InternalConstantCommand(boolean finished, String name, Runnable action, Runnable initialization, Runnable completion, Runnable interruption, SubsystemBase... required) {
			super(); // Removed name for 2020.
			_finished = finished;
			_action = action;
			_initialization = initialization;
			_completion = completion;
			_interruption = interruption;
			if (required != null) {
				for (SubsystemBase s : required) {
					if (s != null) {
						addRequirements(s);
					}
				}
			}
		}
		
		@Override
		public void initialize() {
			if (_initialization != null) {
				_initialization.run();
			}
		}
		
		@Override
		public void execute() {
			if (_action != null) {
				_action.run();
			}
		}
		
		@Override
		public void end(boolean interrupted) {
			if (_completion != null) {
				_completion.run();
			}
		}

		// Removed for 2020.
		// @Override protected void interrupted() { if (_interruption != null) { _interruption.run(); } }
		
		@Override
		public boolean isFinished() {
			return _finished;
		}
	}
	
	/**
	 * Internal Command Extension
	 * <p>
	 * This class extends the original internal command extension which holds the runnable objects, but allows
	 * for an additional override of the isFinished() method, instead of holding a constant, to be based on a
	 * timer.
	 */
	private static class InternalTimedCommand extends InternalConstantCommand {
		
		InternalTimedCommand(String name, double timeout, Runnable action, Runnable initialization, Runnable completion, Runnable interruption, SubsystemBase... required) {
			super(false, name, action, initialization, completion, interruption, required);
			// Removed for 2020.
			// setTimeout(timeout);
		}
		
		@Override
		public boolean isFinished() {
			return false;
			// Removed for 2020.
			// return isTimedOut();
		}
	}
	
	/**
	 * Internal Conditional Command Extension
	 * <p>
	 * The class encapsulates holding a boolean supplier for providing the condition for determining
	 * which command branch to initiate.
	 */
	private static class InternalConditionalCommand extends ConditionalCommand {
		private final BooleanSupplier _condition;
		
		InternalConditionalCommand(String name, BooleanSupplier condition, CommandBase whenTrue, CommandBase whenFalse, SubsystemBase... required) {
			super(whenTrue, whenFalse, condition);
			Objects.requireNonNull(condition);
			_condition = condition;
			if (required != null) {
				for (SubsystemBase s : required) {
					if (s != null) {
						addRequirements(s);
					}
				}
			}
		}
		
		// Removed for 2020.
		// @Override protected boolean condition() { return _condition.getAsBoolean(); }
	}
	
	/**
	 * Command Group Parallel Builder
	 *
	 * @param group  the command group
	 * @param commands  the commands to add in parallel
	 * @return the input group after the commands have been added in parallel
	 */
	private static ParallelCommandGroup parallel(ParallelCommandGroup group, CommandBase... commands) {
		for (CommandBase c : commands) {
			group.addCommands(c);
		}
		return group;
	}
	
	/**
	 * Command Group Sequential Builder
	 *
	 * @param group  the command group
	 * @param commands  the commands to add in sequence
	 * @return the input group after the commands have been added in sequence
	 */
	private static SequentialCommandGroup sequential(SequentialCommandGroup group, CommandBase... commands) {
		for (CommandBase c : commands) {
			group.addCommands(c);
		}
		return group;
	}

}
