package frc.robot;
import java.util.TimerTask;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
 
/*
 * source https://github.com/RobotCasserole1736/RobotCasserole2016/blob/master/RobotCasserole2016/src/org/usfirst/frc/team1736/robot/LIDAR.java
 */

public class LIDAR { //We don't need any pid system, So I took out the code where LIDAR inherits from a PID system
	private I2C i2c;
	private byte[] distance;
	private java.util.Timer updater;
	private final int LIDAR_ADDR = 0x62;
	private final int LIDAR_CONFIG_REGISTER = 0x00;
	private final int LIDAR_DISTANCE_REGISTER = 0x8f;
	
	public LIDAR() {
		i2c = new I2C(Port.kMXP, LIDAR_ADDR);
		distance = new byte[2];
		updater = new java.util.Timer();
	}
	
	/**
	 *  Internally return Distance in cm
	 * @return distance in cm
	 */
	private int getDistance() { //private cuz I don't want people interacting directly with the buffer...yah...
		return (int)Integer.toUnsignedLong(distance[0] << 8) + Byte.toUnsignedInt(distance[1]);
	}
	
	/**
	 * Return Distance in Inches
	 * @return distance in inches
	 */
	public double getDistanceIn() { //I made this function better. It used to be part of a PID system. We didn't need a PID system.
		return getDistance()*0.393701; //inches cuz Merica.
    }
    
	/**
	 * Return Distance in Meters
	 * @return distance in Meters
	 */
	public double getDistanceMeters() { 
		return getDistance()*0.01;
	}
	
	/**
	 *  Start 10Hz polling of LIDAR sensor, in a background task. Only allow 10 Hz. poling.
	 */
	public void start() {
		updater.scheduleAtFixedRate(new LIDARUpdater(), 0, 100);
	}
	
    /**
     * Stop the background sensor-poling task.
     */
	public void stop() {
		updater.cancel();
		updater = new java.util.Timer();
	}
	
	/**
	 * Read from the sensor and update the internal "distance" variable with the result.
	 */
	public void update() {
		i2c.write(LIDAR_CONFIG_REGISTER, 0x04); // Initiate measurement
		Timer.delay(0.04); // Delay for measurement to be taken
		i2c.read(LIDAR_DISTANCE_REGISTER, 2, distance); // Read in measurement
		Timer.delay(0.005); // Delay to prevent over polling
	}
	
	/**
	 *  Timer task to keep distance updated
	 * @author Chris Gerth
	 */
	private class LIDARUpdater extends TimerTask {
		@Override
		public void run() {
			while(true) {
				update();
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
}