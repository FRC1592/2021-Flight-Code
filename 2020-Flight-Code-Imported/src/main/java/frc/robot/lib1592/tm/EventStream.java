package frc.robot.lib1592.tm;

import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Path;
import java.time.Instant;
import java.time.ZonedDateTime;

/**
 * Event Telemetry Stream
 * <p>
 * Implements a telemetry stream for a CSV event file. Reduces the default update rate
 * to 0.5 seconds in order to eliminate excessive overhead. This can be adjusted using
 * {@link #setPeriod(long)} as needed.
 */
public final class EventStream extends FileStream {

	//===============================//
	//      Internal Components      //
	//===============================//
	
	private final Charset charset;



	//=======================//
	//      Constructor      //
	//=======================//
	
	/**
	 * Standard Constructor
	 *
	 * @param folder  the folder for the CSV output file
	 * @param root  the root name of the CSV output file
	 */
	public EventStream(Path folder, String root) {
		super(folder, root, "csv");
		setPeriod(500);
		charset = Charset.forName("UTF-8");
	}



	//===================================//
	//      Event Supplier Methods       //
	//===================================//
	
	/**
	 * Returns whether the stream is initialized.
	 *
	 * @return whether the stream is initialized.
	 */
	public boolean isInitialized() {
		return initTime != null;
	}
	
	/**
	 * Records the specified event to the file. Uses the current time and computes
	 * a relative time from when the event stream was initialized. This method only works
	 * once the stream is initialized (i.e {@link #isInitialized()} is true). Otherwise, 
	 * this is a no-op.
	 *
	 * @param event  the string to record as an event
	 */
	public void recordEvent(String event) {
		if (!isInitialized() || getFileChannel()==null) {return;}
		try {
			addStringToBuffer(Double.toString((Instant.now().toEpochMilli()-initTime)/1000d) + ", " + event + "\r\n");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}



	//=============================================//
	//      Package Private Methods (Stream)       //
	//=============================================//
	
	private Long initTime = null;
	
	/** Internal Accessor */
	private void addStringToBuffer(String in) throws IOException {
		addToBuffer(in.getBytes(charset));
	}
	
	@Override void initializeStream(ZonedDateTime dateTime) throws IOException {
		super.initializeStream(dateTime);
		if (getFileChannel()!=null) {
			initTime = dateTime.toInstant().toEpochMilli();
			addStringToBuffer("Time (s), Event\r\n");
		}
	}
	
	@Override void updateStream() throws IOException {
		if (getFileChannel()!=null) {
			//forceBuffer(); Performance Savings
		}
	}
	
	@Override void pauseStream() throws IOException {
		if (getFileChannel()!=null) {
			forceBuffer();
		}
		super.pauseStream();
	}
	
	@Override void continueStream() throws IOException {}
	
	@Override void finalizeStream() throws IOException {
		if (getFileChannel()!=null) {
			forceBuffer();
		}
		super.finalizeStream();
	}

}
