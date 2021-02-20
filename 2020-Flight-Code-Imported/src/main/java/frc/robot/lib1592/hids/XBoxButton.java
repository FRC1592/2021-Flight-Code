package frc.robot.lib1592.hids;

// import org.frc1592.lib1592.trigger.TriggerFactory.Response;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
Left Trigger (Back)                   Right Trigger (Back)

   _.-'BUMP `-._                          _,-'BUMP'-._
,-'             `-.,__________________,.-'      .-.    `-.
/      .-Y .                ___                ( Y )      \
/    ,' .-. `.      ____   / X \   _____    .-. `-` .-.    \
/   -X |   | +X    (Back) | / \ | (Start)  ( X )   ( B )   |
/    `. `-' ,'    __       \___/            `-` ,-. `-`    |
|      `+Y `   ,-`  `-.          .-Y .         ( A )       |
|             / -'  `- \       ,'  .  `.        `-`        |
|            |    POV   |     -X -  - +X                   |
!             \ -.  ,- /       `.  '  ,'                   |
|              `-.__,-'          `+Y `                     |
|                  ________________                        /
|             _,-'`                ``-._                  /
|          ,-'                          `-.              /  Based on 10/10 ASCII ART BYosrevad
\       ,'                                 `.           /
 `.__,-'                                     `-.______,'     */


public class XBoxButton extends JoystickButton {

	public enum ButtonName {
		A(1),
		B(2),
		X(3),
		Y(4),
		LEFT_BUMPER(5),
		RIGHT_BUMPER(6),
		BACK(7),
		START(8),
		LEFT_STICK(9),
		RIGHT_STICK(10),
		LEFT_TRIGGER(11),
		RIGHT_TRIGGER(12);

		public final int value;

		private ButtonName(int button) {
			this.value = button;
		}

	}

	public XBoxButton(GenericHID joystick, int buttonNumber) {
		super(joystick, buttonNumber);
	}
	
	public XBoxButton(GenericHID joystick, ButtonName buttonName) {
		super(joystick, buttonName.value);
	}
	
	public static XBoxButton of(GenericHID joystick, int buttonNumber) {
		return new XBoxButton(joystick, buttonNumber);
	}
	
	public static XBoxButton of(GenericHID joystick, ButtonName buttonName) {
		return new XBoxButton(joystick, buttonName);
	}
	
	// public static XBoxButton of(GenericHID joystick, int buttonNumber, Response... responses) {
	// 	XBoxButton out = of(joystick, buttonNumber);
	// 	if (responses != null) {
	// 		for (Response r : responses) {
	// 			if (r != null) {
	// 				r.applyTo(out);
	// 			}
	// 		}
	// 	}
	// 	return out;
	// }
	
	// public static XBoxButton of(GenericHID joystick, ButtonName buttonName, Response... responses) {
	// 	return of(joystick, buttonName.value, responses);
	// }
	
	
}
