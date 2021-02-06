package frc.robot.lib1592.selfTest;

import java.util.function.BooleanSupplier;

public class BooleanComparison extends TestBase {
	BooleanSupplier src;
	boolean target;
	
	public BooleanComparison(String name, BooleanSupplier src, boolean target) {
		super(name);
		
		this.src = src;
		this.target = target;
	}

	@Override
	protected boolean doTest() {
		StringBuilder result = new StringBuilder();
		result.append(src.getAsBoolean());
		result.append(" equals ");
		result.append(target);
		
		reportResult(src.getAsBoolean() == target, result.toString());
		return src.getAsBoolean() == target;
	}
}