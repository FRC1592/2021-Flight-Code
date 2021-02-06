package frc.robot.lib1592.trigger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Static Class For Building Basic Triggers
 * <p>
 * This builder contains all of the basic methods for generating triggers in an in-line manner
 * with most of the basic options available for construction. Advanced triggering or maintenance of
 * state in a trigger should be handled by an individual class extended from {@link Trigger}.
 */
public class TriggerFactory {
	
	/** Static Class */
	private TriggerFactory() {}
	
	
	
	//=====================//
	//   Base Variations   //
	//=====================//
	
	/**
	 * Generates a trigger which uses the input supplier as the trigger state.
	 *
	 * @param state  the trigger state supplier
	 * @return the generated trigger
	 * @throws IllegalArgumentException if state is null
	 */
	public static Trigger of(BooleanSupplier state) {
		return new InternalTrigger(state);
	}
	
	/**
	 * Generates a trigger which uses the input supplier as the trigger state.
	 * Additionally, one or more responses can be provided to in-line the assignment
	 * of trigger actions.
	 * <p>
	 * The trigger responses can be null or contain null elements, in which case the
	 * null objects are ignored and simply just not added to the trigger actions
	 *
	 * @param state  the trigger state supplier
	 * @param responses  the trigger actions
	 * @return the generated trigger
	 * @throws IllegalArgumentException if state is null
	 */
	public static Trigger of(BooleanSupplier state, Response... responses) {
		Trigger out = new InternalTrigger(state);
		if (responses != null) {
			for (Response r : responses) {
				if (r != null) {
					r.applyTo(out);
				}
			}
		}
		return out;
	}
	
	
	
	//====================//
	//   Response Class   //
	//====================//
	
	/**
	 * Trigger Response Class
	 * <p>
	 * This class encapsulates one or more Command(s) with the type of trigger 
	 * action that would generate an action to be performed on the command(s). 
	 * This class is used to populate the trigger's actions instead of calling 
	 * the specific methods on a trigger after creation. While not required, 
	 * this can be used to compartmentalize the code and further enhance the 
	 * trigger in-lining.
	 */
	public static class Response {
		private final Action _action;
		private final List<Command> _command;
		
		/** Internal Constructor */
		private Response(Action action, List<Command> command) {
			Objects.requireNonNull(action);
			_action = action;
			_command = new ArrayList<>();
			if (command != null) {
				for (Command c : command) {
					if (c != null) {
						_command.add(c);
					}
				}
			}
		}
		
		/**
		 * Applies this response to the input trigger.
		 *
		 * @param trigger  the trigger to apply to
		 * @throws NullPointerException if trigger is null
		 */
		public void applyTo(Trigger trigger) {
			for (Command c : _command) {
				_action.execute(trigger, c);
			}
		}
		
		/**
		 * Returns a trigger response using the specified action to trigger the
		 * specified commands.
		 * <p>
		 * The commands input can be null, in which case no commands will be used in this
		 * response. Additionally, elements of the commands array can also be null, in
		 * which case they are ignored.
		 *
		 * @param action  the response action type
		 * @param commands  the response commands
		 * @return the trigger response
		 * @throws NullPointerException if action is null
		 */
		public static Response of(Action action, Command... commands) {
			return new Response(action, commands == null ? null : Arrays.asList(commands));
		}
	}
	
	/**
	 * Trigger/Button Action Enumeration
	 * <p>
	 * This enumeration encapsulates the different options for how to attach
	 * a Command to a trigger and therefore what trigger actions cause what
	 * responses by a command.
	 */
	public static enum Action {
		WHEN_ACTIVE((t, c) -> {t.whenActive(c);}),
		WHEN_PRESSED((t, c) -> {t.whenActive(c);}),
		WHILE_ACTIVE((t, c) -> {t.whileActiveContinuous(c);}),
		WHILE_HELD((t, c) -> {t.whileActiveContinuous(c);}), // TODO: Add other wileActive case
		WHEN_INACTIVE((t, c) -> {t.whenInactive(c);}),
		WHEN_RELEASED((t, c) -> {t.whenInactive(c);}),
		TOGGLE_WHEN_ACTIVE((t, c) -> {t.toggleWhenActive(c);}),
		TOGGLE_WHEN_PRESSED((t, c) -> {t.toggleWhenActive(c);}),
		CANCEL_WHEN_ACTIVE((t, c) -> {t.cancelWhenActive(c);}),
		CANCEL_WHEN_PRESSED((t, c) -> {t.cancelWhenActive(c);});
		
		private final BiConsumer<Trigger, Command> _method;
		
		/** Internal Constructor **/
		private Action(BiConsumer<Trigger, Command> method) {
			_method = method;
		}
		
		/**
		 * Internal Execution to apply the command to the trigger for the specified action
		 *
		 * @param trigger  the trigger to apply to
		 * @param command  the command to apply
		 */
		void execute(Trigger trigger, Command command) {
			if (_method == null || trigger == null || command == null) { return; }
			_method.accept(trigger, command);
		}
	}
	
	
	
	//=============================//
	//   Internal Implementation   //
	//=============================//
	
	/**
	 * Internal Trigger Extension
	 * <p>
	 * This class encapsulates holding an input boolean supplier for usage as
	 * the response to the trigger state get() call.
	 */
	private static class InternalTrigger extends Trigger {
		private final BooleanSupplier _state;
		
		InternalTrigger(BooleanSupplier state) {
			_state = state;
		}
		
		@Override public boolean get() { return _state.getAsBoolean(); }
	}

}
