package frc.robot.lib1592.tm;

import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.file.OpenOption;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * File Telemetry Stream
 * <p>
 * Implements telemetry stream using {@link FileChannel} under the hood.
 */
public abstract class FileStream extends TelemetryStream {

	//===============================//
	//      Internal Components      //
	//===============================//

	private Path folder;
	private String root;
	private String ext;
	private FileOutputStream stream;
	private FileChannel channel;
	private final ByteBuffer streamBuffer;



	//=======================//
	//      Constructor      //
	//=======================//

	/** Protected Constructor : Buffer 16384 bytes in native Byte order */
	protected FileStream(Path folder, String root, String extension) {
		this(folder, root, extension, 16384);
	}
	
	/** Protected Constructor : Buffer specified bytes in native Byte order */
	protected FileStream(Path folder, String root, String extension, int capacity) {
		this(folder, root, extension, capacity, ByteOrder.nativeOrder());
	}
	
	/** Protected Constructor */
	protected FileStream(Path folder, String root, String extension, int capacity, ByteOrder order) {
		this.folder = folder;
		this.root = root;
		this.ext = extension;
		streamBuffer = ByteBuffer.allocateDirect(16384);
		streamBuffer.order(order);
	}
	
	/**
	 * Returns the FileChannel on which to write in {@link #updateStream()}. The
	 * returned channel can be null either because the stream has not been initialized
	 * or because a failure occurred during initialization.
	 *
	 * @return the file channel to write to
	 */
	protected FileChannel getFileChannel() {
		return channel;
	}



	//=============================================//
	//      Package Private Methods (Stream)       //
	//=============================================//
	
	@Override void initializeStream(ZonedDateTime dateTime) throws IOException {
		DriverStation ds = DriverStation.getInstance();
		String filename = String.join("_", ds.getMatchType().toString()+Integer.toString(ds.getMatchNumber()), root, dateTime.format(DateTimeFormatter.ofPattern("uu_MM_dd_HH_mm_ss"))) + "." + ext;
		Set<OpenOption> opts = new HashSet<>();
		opts.add(StandardOpenOption.CREATE);
		opts.add(StandardOpenOption.TRUNCATE_EXISTING);
		try {
			stream = new FileOutputStream(folder.resolve(filename).toString());
			channel = stream.getChannel();
		} catch (IOException e) {
			channel = null;
			e.printStackTrace();
		}
	}
	
	@Override void pauseStream() throws IOException {
		if (channel!=null) {
			try {
				channel.force(true);
			} catch (IOException e) {}
		}
	}
	
	@Override void finalizeStream() throws IOException {
		if (channel!=null) {
			try {
				channel.force(true);
			} catch (IOException e) {}
			try {
				channel.close();
				stream.close();
			} catch (IOException e1) {}
		}
		super.finalizeStream();
	}
	
	/** Protected Accessor */
	protected void addToBuffer(byte[] inBytes) throws IOException {
		if (streamBuffer.remaining() < inBytes.length) {
			forceBuffer();
		}
		streamBuffer.put(inBytes);
	}
	
	/** Protected Accessor */
	protected void addToBuffer(ByteBuffer in) throws IOException {
		if (streamBuffer.remaining() < in.remaining()) {
			forceBuffer();
		}
		streamBuffer.put(in);
	}
	
	/** Private Accessor */
	protected void forceBuffer() throws IOException {
		if (streamBuffer.position()==0) {return;}
		streamBuffer.flip();
		if (channel!=null) {
			channel.write(streamBuffer);
		}
		streamBuffer.clear();
	}
	

}
