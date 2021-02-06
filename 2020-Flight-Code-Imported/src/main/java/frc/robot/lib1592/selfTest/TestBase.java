package frc.robot.lib1592.selfTest;

import java.util.ArrayList;
import java.util.List;

public abstract class TestBase implements Runnable {
	private String name;
	private boolean passed = false;
	List<String> failures = new ArrayList<String>();
	
	public TestBase(String name) {
		this.name = name;
	}
	
	@Override
	public void run() {
		passed = doTest();
	}
	
	public String getName() {
		return name;
	}
	
	public boolean hasPassed() {
		return passed;
	}
	
	public List<String> getFailures() {
		return failures;
	}
	
	protected abstract boolean doTest();
	
	protected void reportResult(boolean success, String successReason) {
		String result = "\t\t[" + (success ? "*" : "!") + "] Test case '" + name + "' " + (success ? "passed" : "potatoed") + " " + successReason;
		System.out.println(result);
		if(!success)
			failures.add(name);
	}
}
