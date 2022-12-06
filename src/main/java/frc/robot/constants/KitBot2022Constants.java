package frc.robot.constants;

public class KitBot2022Constants implements Constants {
  @Override
  public RobotType robot() {
    return RobotType.ROBOT_KITBOT;
  }

  @Override
  public String logFolder() {
    return "/dev/sda1";
  }
}
