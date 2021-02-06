package frc.robot.lib1592.selfTest;

import java.util.List;

import frc.robot.lib1592.RobotUtility;

public class TestBuilder extends TestGroup {
	
	public TestBuilder(String subsystemName) {
		super(subsystemName);
		System.out.println();
	}
	
	public void runTests() {
		RobotUtility.printDebugHeader("Testing " + name + " subsystem");
		boolean passed = true;

		for(Runnable action:actions) {
			if(dangerousActions.contains(action))
				if(passed)
					action.run(); //Run dangerous test only if all prerequisites passed
				else
					System.out.println("[!!] Test " + ((TestBase)action).getName() + " not performed due to unsafe conditions!");
			else
				action.run();
			if(action instanceof TestBase)
				passed &= ((TestBase)action).hasPassed();
		}
		RobotUtility.printDebugHeader(name + (passed?" has PASSED all tests":" has FAILED one or more tests!"));

		createReport(getFailures());
	}
	
	private void createReport(List<String> failures) {
		System.out.println();
		System.out.println();
		System.out.println();
		RobotUtility.printDebugHeader("Self test results");
		if(failures.size() == 0)
			System.out.println("\tAll tests completed SUCCESSFULLY!");
		else {
			for(String fail:failures) {
				System.out.println("\t "+fail+" FAILED!");
			}
		}
	}
}
