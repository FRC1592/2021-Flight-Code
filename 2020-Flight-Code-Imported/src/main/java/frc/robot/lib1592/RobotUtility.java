package frc.robot.lib1592;

import java.util.Collection;
import java.util.Collections;

/**
 * Static Utility Class
 */
public final class RobotUtility {
	
	/** Static Class */
	private RobotUtility() {};
	
	
	
	//==========================//
	//      String Methods      //
	//==========================//
	
	/**
	 * Ensures that the resulting string from this method is a unique string which is not
	 * contained in the supplied collection. This method will first simply return the input string
	 * if it is not found in the collection. After that, additional actions are taken until a
	 * string is generated that is unique to the collection. The string is NOT added to the 
	 * collection in this method.
	 *
	 * @param in  the string input
	 * @param collection  the collection to test against for uniqueness
	 * @return the original string, if not in the collection, or a generated string based off of the
	 * 			input string that would be unique in the collection
	 */
	public static final String ensureUnique(String in, Collection< ? extends String> collection) {
		if (collection==null) {collection = Collections.emptySet();}
		if (in==null) {in = "";}
		if (collection.contains(in)) {
			long v = 1;
			String nameMod = in+"_"+Long.toString(v++);
			while (collection.contains(nameMod)) {
				nameMod = in+"_"+Long.toString(v++);
			}
			in = nameMod;
		}
		return in;
	}
	
	public static final void printDebugHeader(String header) {
		printDebugHeader(header, 0);
	}
	
	public static final void printDebugHeader(String header, int indents) {
		System.out.println(repeat("\t", indents) + "//======" + repeat("=", header.length()) + "======//");
		System.out.println(repeat("\t", indents) + "//      " + header + "      //");
		System.out.println(repeat("\t", indents) + "//======" + repeat("=", header.length()) + "======//");
	}

	/**
	 * Because Java sucks I had to make this function, it repeats a string num times
	 * @param str String to repeat
	 * @param num Number of times to repeat it
	 * @return  The repeated string
	 */
	public static final String repeat(String str, int num) {
	StringBuilder builder = new StringBuilder();
	for(int i = 0; i < num; i++) {
		builder.append(str);
	}
	return builder.toString();
}
}
