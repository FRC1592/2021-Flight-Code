package frc.robot.lib1592.selfTest;

public class ComparisonParams {
	public ComparisonType type;
	public Number tolerance;
	
	public ComparisonParams(ComparisonType type, Number tolerance) {
		this.type = type;
		this.tolerance = tolerance;
	}
	
	public ComparisonParams(ComparisonType type) {
		this(type, null);
	}

	public enum ComparisonType {
		EQUAL, LESS, GREATER, TOLERANCE,
		NOT_EQUAL, OUTSIDE_TOLERANCE
	}
}
