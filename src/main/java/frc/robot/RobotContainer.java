package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.controllers.CustomController2022;
import lib.input.ControllerQueue;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Command autoCommand = new PrintCommand("Auto!");

  // Driver Controller
  private final CommandXboxController driverJoystick = new CommandXboxController(0);

  // Operator Controller
  private final CustomController2022 operatorJoystick = new CustomController2022(1);
  private final JoystickButton operatorClimberLimits =
      operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_LIMITS);
  private final JoystickButton operatorShooterAuto =
      operatorJoystick.getButton(CustomController2022.Buttons.SHOOTER_AUTO);
  private final JoystickButton operatorEverything =
      operatorJoystick.getButton(CustomController2022.Buttons.EVERYTHING);
  private final JoystickButton operatorShoot =
      operatorJoystick.getButton(CustomController2022.Buttons.SHOOT);
  private final JoystickButton operatorClimberLock =
      operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_LOCK);
  private final JoystickButton operatorClimberUp =
      operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_UP);
  private final JoystickButton operatorClimberDown =
      operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_DOWN);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initializeSubsystems();
    // Configure the button bindings
    configureButtonBindings();
    ControllerQueue.getInstance().addController(operatorJoystick);
  }

  private void initializeSubsystems() {}

  private void configureSubsystems() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Add button to command mappings here.
    // See
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
    configureSubsystems();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
