package frc.robot.lib1592.drivers;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;

/**
 * Driver for a Lidar Lite sensor
 */
public class LidarLiteSensor {
	private I2C mI2C;
	private byte[] mDistance;
	private java.util.Timer mUpdater;
	private boolean mHasSignal;

	private final static int LIDAR_ADDR = 0x62;
	private final static int LIDAR_CONFIG_REGISTER = 0x00;
	private final static int LIDAR_DISTANCE_REGISTER = 0x8f;

	public LidarLiteSensor(Port port) {
		mI2C = new I2C(port, LIDAR_ADDR);
		mDistance = new byte[2];
		mUpdater = new java.util.Timer();
		mHasSignal = false;
	}

	/**
	 * @return Distance in meters
	 */
	public double getDistance() {
		int distCm = (int) Integer.toUnsignedLong(mDistance[0] << 8) + Byte.toUnsignedInt(mDistance[1]);
		return distCm / 100.0;
	}

	/**
	 * @return true iff the sensor successfully provided data last loop
	 */
	public boolean hasSignal() {
		return mHasSignal;
	}

	/**
	 * Start 10Hz polling
	 */
	public void start() {
		start(100);
	}

	/**
	 * Start polling for period in milliseconds
	 */
	public void start(int period) {
		TimerTask task = new TimerTask() {
			@Override
			public void run() {
				update();
			}
		};
		mUpdater.scheduleAtFixedRate(task, 0, period);
	}

	public void stop() {
		mUpdater.cancel();
		mUpdater = new java.util.Timer();
	}

	private void update() {
		if (mI2C.write(LIDAR_CONFIG_REGISTER, 0x04)) {
			// the write failed to ack
			mHasSignal = false;
			return;
		}
		Timer.delay(0.04); // Delay for measurement to be taken
		if (!mI2C.read(LIDAR_DISTANCE_REGISTER, 2, mDistance)) {
			// the read failed
			mHasSignal = false;
			return;
		}
		mHasSignal = true;
		Timer.delay(0.005); // Delay to prevent over polling
	}

	/**
	 * Perform an instantaneous sample of the LIDAR sensor
	 * @return distance in cm
	 */
	public int sampleDistance()//
	{
		mI2C.write(LIDAR_CONFIG_REGISTER, 0x04);

		byte[] buffer=new byte[1];
		buffer[0]=0x01;
		byte[] nakbuffer=new byte[1];
		mI2C.writeBulk(buffer);
		mI2C.readOnly(nakbuffer, 1);//nak
		int hex=Byte.toUnsignedInt(nakbuffer[0]);
		while ((hex&0x01)!=0)
		{
			mI2C.readOnly(nakbuffer, 1);//nak
			hex=Byte.toUnsignedInt(nakbuffer[0]);
		}
		buffer[0]=(byte) LIDAR_DISTANCE_REGISTER;
		mI2C.writeBulk(buffer);

		byte [] readbuffer=new byte[2];
		mI2C.readOnly(readbuffer,2);
		Timer.delay(.005);

		int dist = (int)(Integer.toUnsignedLong(readbuffer[0] << 8)) + Byte.toUnsignedInt(readbuffer[1]);
		int lastValidDist=-1;
		if (dist > 0)
		{
			lastValidDist = dist;
		}

		return 	lastValidDist;
	}
	public int sampleDistanceInches()
	{
		return (int) (this.sampleDistance()+.5/2.54);
	}

}
