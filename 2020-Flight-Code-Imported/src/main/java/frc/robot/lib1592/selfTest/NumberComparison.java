package frc.robot.lib1592.selfTest;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import frc.robot.lib1592.selfTest.ComparisonParams.ComparisonType;

public class NumberComparison extends TestBase {
	private final int AVERAGES = 3;
	
	DoubleSupplier src;
	ComparisonParams params;
	double[] targets;
	
	public NumberComparison(String name, DoubleSupplier src, ComparisonParams params, double... targets) {
		super(name);
		
		this.src = src;
		this.params = params;
		this.targets = targets;
	}

	@Override
	protected boolean doTest() {
		double val = 0;
		boolean atleastOnePassed = false;
		
		for(int i=0; i < AVERAGES; i++)
			val += src.getAsDouble();
		val /= AVERAGES;
		
		for(double tgt:targets) {
			switch(params.type) {
			case EQUAL:
			case NOT_EQUAL:
				if(val == tgt)
					atleastOnePassed |= params.type == ComparisonType.EQUAL;
				else
					atleastOnePassed |= params.type == ComparisonType.NOT_EQUAL;
				break;
			case LESS:
				if(val < tgt)
					atleastOnePassed = true;
				break;
			case GREATER:
				if(val > tgt)
					atleastOnePassed = true;
				break;
			case TOLERANCE:
			case OUTSIDE_TOLERANCE:
				if((tgt + (double)params.tolerance) > val && val > (tgt - (double)params.tolerance))
					atleastOnePassed |= params.type == ComparisonType.TOLERANCE;
				else
					atleastOnePassed |= params.type == ComparisonType.OUTSIDE_TOLERANCE;
				break;
			}
		}
		
		StringBuilder result = new StringBuilder();
		result.append(src.getAsDouble());
		switch(params.type) {
		case LESS:
			result.append(" less than ");
			break;
		case GREATER:
			result.append(" greater than ");
			break;
		case EQUAL:
			result.append(" is equal to ");
			break;
		case NOT_EQUAL:
			result.append(" is not equal to ");
			break;
		case TOLERANCE:
			result.append(" is within +/- ");
			result.append(params.tolerance);
			result.append(" of ");
			break;
		case OUTSIDE_TOLERANCE:
			result.append(" is not within +/- ");
			result.append(params.tolerance);
			result.append(" of ");
			break;
		}
		
		result.append(Arrays.toString(targets));
		reportResult(atleastOnePassed, result.toString());
		return atleastOnePassed;
	}
}