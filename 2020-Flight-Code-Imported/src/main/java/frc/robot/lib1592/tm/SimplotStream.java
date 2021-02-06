package frc.robot.lib1592.tm;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.Charset;
import java.nio.file.Path;
import java.time.Instant;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.function.DoubleSupplier;

import frc.robot.lib1592.RobotUtility;

/**
 * Simplot Telemetry Stream
 * <p>
 * Implements a telemetry stream for a Simplot HPL file.
 */
public final class SimplotStream extends FileStream {

	//===============================//
	//      Internal Components      //
	//===============================//
	
	private final boolean isFloat;
	private final Charset charset;
	private final ByteBuffer buffer;
	private final SortedMap<String, DoubleSupplier> dataMap;



	//=======================//
	//      Constructor      //
	//=======================//
	
	/**
	 * Standard Constructor
	 * <p>
	 * This version stores all data as floats (4 byte) for space and performance reasons.
	 *
	 * @param folder  the folder for the Simplot output file
	 * @param root  the root name of the Simplot output file
	 */
	public SimplotStream(Path folder, String root) {
		this(folder, root, true);
	}

	/**
	 * Expanded Constructor
	 *
	 * @param folder  the folder for the Simplot output file
	 * @param root  the root name of the Simplot output file
	 * @param isFloat  logical specifying whether to save the data as floats (4 byte) or doubles (8 byte)
	 */
	public SimplotStream(Path folder, String root, boolean isFloat) {
		super(folder, root, "hpl", 16384, ByteOrder.LITTLE_ENDIAN);
		this.isFloat = isFloat;
		charset = Charset.forName("UTF-8");
		dataMap = new TreeMap<>();
		buffer = ByteBuffer.allocate(4096);
		buffer.order(ByteOrder.LITTLE_ENDIAN);
	}



	//==================================//
	//      Data Supplier Methods       //
	//==================================//
	
	/**
	 * Returns an unmodifiable view of the data supplier map.
	 */
	public final SortedMap<String, DoubleSupplier> getUnmodifiableDataSupplierMap() {return Collections.unmodifiableSortedMap(dataMap);}
	
	/**
	 * Add a data supplier to the supplier map. If the supplier already exists in the map,
	 * the current name of that supplier in the map will be returned. Otherwise the supplier
	 * will be added to the map. A name will be generated for the supplier and will be returned.
	 *
	 * @param supplier  the data supplier
	 * @return the name actually used for the supplier in the map
	 */
	public final String addDataSupplier(DoubleSupplier supplier) {
		if (supplier==null) {throw new NullPointerException();}
		if (dataMap.containsValue(supplier)) {
			for (Map.Entry<String, DoubleSupplier> e : dataMap.entrySet()) {
				if (e.getValue().equals(supplier)) {
					return e.getKey();
				}
			}
		}
		String name = RobotUtility.ensureUnique(null, dataMap.keySet());
		dataMap.put(name, supplier);
		return name;
	}
	
	/**
	 * Add a data supplier to the supplier map. If the supplier already exists in the map,
	 * the current name of that supplier in the map will be returned. Otherwise the supplier
	 * will be added to the map. It will be added under the input name, if that name does not
	 * already exist in the map, otherwise a new name will be produced so that it can be added.
	 * The name actually used to insert it into the map is returned.
	 *
	 * @param name  the recommended name
	 * @param supplier  the data supplier
	 * @return the name actually used for the supplier in the map
	 */
	public final String addDataSupplier(String name, DoubleSupplier supplier) {
		if (supplier==null) {throw new NullPointerException();}
		if (dataMap.containsValue(supplier)) {
			for (Map.Entry<String, DoubleSupplier> e : dataMap.entrySet()) {
				if (e.getValue().equals(supplier)) {
					return e.getKey();
				}
			}
		}
		name = RobotUtility.ensureUnique(name, dataMap.keySet());
		dataMap.put(name, supplier);
		return name;
	}
	
	/**
	 * Removes the data supplier from the supplier map with the input name, if it exists.
	 *
	 * @param name  the supplier name
	 * @return the data supplier removed, or null if no supplier is removed
	 */
	public final DoubleSupplier removeDataSupplier(String name) {
		return dataMap.remove(name);
	}



	//=============================================//
	//      Package Private Methods (Stream)       //
	//=============================================//
	
	@Override void initializeStream(ZonedDateTime dateTime) throws IOException {
		super.initializeStream(dateTime);
		if (getFileChannel()!=null) {
			long initTime = dateTime.toInstant().toEpochMilli();
			SortedMap<String, DoubleSupplier> data = getUnmodifiableDataSupplierMap();
			if (data.get("Time")==null) {
				this.addDataSupplier("Time", ()->{return (Instant.now().toEpochMilli() - initTime) / 1000d;});
			}
			buffer.putInt(data.size());
			buffer.putInt(0);
			buffer.putInt(isFloat ? 4 : 8);
			
			String info = "Generated Telemetry :: SimplotStream";
			byte[] infoBytes = info.getBytes(charset);
			List<byte[]> labelBytes = new ArrayList<>();
			int labelMax = 0;
			for (String s : data.keySet()) {
				byte[] bytes = s.getBytes(charset);
				labelBytes.add(bytes);
				labelMax = Math.max(labelMax, bytes.length);
			}
			
			buffer.putInt(labelMax);
			buffer.putInt(infoBytes.length);
			buffer.put(infoBytes);
			for (byte[] b : labelBytes) {
				buffer.put(b);
				if (b.length<labelMax) {
					buffer.position(buffer.position()+(labelMax-b.length));
				}
			}
			buffer.flip();
			addToBuffer(buffer);
		}
	}
	
	@Override void updateStream() throws IOException {
		if (getFileChannel()!=null) {
			buffer.clear();
			SortedMap<String, DoubleSupplier> data = getUnmodifiableDataSupplierMap();
			for (DoubleSupplier v : data.values()) {
				if (isFloat) {
					buffer.putFloat((float) v.getAsDouble());
				} else {
					buffer.putDouble(v.getAsDouble());
				}
			}
			buffer.flip();
			addToBuffer(buffer);
		}
	}
	
	@Override void pauseStream() throws IOException {
		if (getFileChannel()!=null) {
			finalizeSimplot();
		}
		super.pauseStream();
	}
	
	@Override void continueStream() throws IOException {
		if (getFileChannel()!=null) {
			long offset = 0;
			if (isFloat) {
				offset = Float.BYTES*getUnmodifiableDataSupplierMap().size();
			} else {
				offset = Double.BYTES*getUnmodifiableDataSupplierMap().size();
			}
			getFileChannel().position(getFileChannel().position()-offset);
		}
	}
	
	@Override void finalizeStream() throws IOException {
		if (getFileChannel()!=null) {
			finalizeSimplot();
		}
		super.finalizeStream();
	}
	
	private void finalizeSimplot() throws IOException {
		buffer.clear();
		SortedMap<String, DoubleSupplier> data = getUnmodifiableDataSupplierMap();
		for (int i=0; i<data.size(); i++) {
			if (isFloat) {
				buffer.putFloat((float) 1234567.0);
			} else {
				buffer.putDouble(1234567.0);
			}
		}
		buffer.flip();
		addToBuffer(buffer);
		forceBuffer();
	}

}
