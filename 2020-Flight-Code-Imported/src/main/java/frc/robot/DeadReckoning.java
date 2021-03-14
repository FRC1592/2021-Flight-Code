package frc.robot;

public class DeadReckoning {
  private final double m_startTime;

  public DeadReckoning(double startTime) {
    m_startTime = startTime;
  }

  public void printCmd(String cmdName, String cmdParams) {
    double elapsedTime = System.currentTimeMillis() - m_startTime;
    System.out.println("new " + cmdName + "(" + cmdParams + ").withTimeout(" + elapsedTime + " * Constants.MS_TO_SEC),");
  }

  public void printDriveForwardCmd() {
    printCmd("DriveForwardConstantSpeed", "m_chassis, " + Constants.AUTO_SPEED_FORWARD);
  }

  public void printRotateCmd(String direction) {
    printCmd("Rotate" + direction, "m_chassis");
  }
}