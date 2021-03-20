package frc.robot;

public class DeadReckoning {
  private double m_startTime;
  private double m_stopTime;
  private double m_elapsedTime;

  public void printCmd(String cmdName, String cmdParams) {
    System.out.println("new " + cmdName + "(" + cmdParams + ").withTimeout(" + m_elapsedTime + " * Constants.MS_TO_SEC),");
    System.out.println("new WaitCommand(1.0),");
  }

  public void printDriveForwardCmd() {
    printCmd("DriveForwardConstantSpeed", "m_chassis, " + Constants.AUTO_SPEED_FORWARD);
  }

  public void printRotateCmd(String direction) {
    printCmd("Rotate" + direction, "m_chassis");
  }

  public void startTimer() {
    m_startTime = System.currentTimeMillis();
  }

  public void stopTimer() {
    m_stopTime = System.currentTimeMillis();
    m_elapsedTime = m_stopTime - m_startTime;
  }
}