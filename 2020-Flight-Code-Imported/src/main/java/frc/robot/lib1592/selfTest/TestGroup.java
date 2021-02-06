package frc.robot.lib1592.selfTest;

import java.util.ArrayList;
import java.util.List;

import frc.robot.lib1592.RobotUtility;

import edu.wpi.first.wpilibj.Timer;

public class TestGroup extends TestBase {
	String name;
	List<Runnable> actions = new ArrayList<Runnable>();
	List<Runnable> dangerousActions = new ArrayList<Runnable>();
	
	public TestGroup(String name) {
		super(name);
		this.name = name;
	}
	
	public void addAction(Runnable action) {
		actions.add(action);
	}
	
	public void addDangerousAction(Runnable action) {
		actions.add(action);
		dangerousActions.add(action);
	}
	
	public void addTest(TestBase test) {
		actions.add(test);
	}
	
	public void addDangerousTest(TestBase test) {
		actions.add(test);
		dangerousActions.add(test);
	}
	
	public void addDelay(int millis) {
		actions.add(() -> Timer.delay(millis/1000d));
	}

	@Override
	protected boolean doTest() {
		boolean passed = true;
		
		//"I broke the code" - Nikhil
		
		RobotUtility.printDebugHeader("Running test group '" + name + "'", 1);
		for(Runnable test:actions) {
			if(dangerousActions.contains(test))
				if(passed)
					test.run(); //Run dangerous test only if all prerequisites passed
				else if(test instanceof TestBase)
					System.out.println("\t\t[!!] Test " + ((TestBase)test).getName() + " not performed due to unsafe conditions!");
				else
					System.out.println("\t\t[!!] Action not performed due to unsafe conditions");
			else
				test.run();
			if(test instanceof TestBase)
				passed &= ((TestBase)test).hasPassed();
				
		}
		RobotUtility.printDebugHeader(name + (passed?" has PASSED all tests":" has FAILED one or more tests!"), 1);
		
		return passed;
	}

}
