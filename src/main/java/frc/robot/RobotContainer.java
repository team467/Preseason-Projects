package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.controllers.CustomController2022;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMAX;
import lib.input.ControllerQueue;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Command autoCommand = new PrintCommand("Auto!");

    private Drive drive;

    // Driver Controller
    private final XboxController driverJoystick = new XboxController(0);
    private final JoystickButton driverButtonA = new JoystickButton(driverJoystick, XboxController.Button.kA.value);
    private final JoystickButton driverButtonB = new JoystickButton(driverJoystick, XboxController.Button.kB.value);
    private final JoystickButton driverButtonX = new JoystickButton(driverJoystick, XboxController.Button.kX.value);
    private final JoystickButton driverButtonY = new JoystickButton(driverJoystick, XboxController.Button.kY.value);
    private final JoystickButton driverButtonBumperLeft = new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverButtonBumperRight = new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value);
    private final JoystickButton driverButtonBack = new JoystickButton(driverJoystick, XboxController.Button.kBack.value);
    private final JoystickButton driverButtonStart = new JoystickButton(driverJoystick, XboxController.Button.kStart.value);
    private final JoystickButton driverButtonStickLeft = new JoystickButton(driverJoystick, XboxController.Button.kLeftStick.value);
    private final JoystickButton driverButtonStickRight = new JoystickButton(driverJoystick, XboxController.Button.kRightStick.value);

    // Operator Controller
    private final CustomController2022 operatorJoystick = new CustomController2022(1);
    private final JoystickButton operatorClimberLimits = operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_LIMITS);
    private final JoystickButton operatorShooterAuto = operatorJoystick.getButton(CustomController2022.Buttons.SHOOTER_AUTO);
    private final JoystickButton operatorEverything = operatorJoystick.getButton(CustomController2022.Buttons.EVERYTHING);
    private final JoystickButton operatorShoot = operatorJoystick.getButton(CustomController2022.Buttons.SHOOT);
    private final JoystickButton operatorClimberLock = operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_LOCK);
    private final JoystickButton operatorClimberUp = operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_UP);
    private final JoystickButton operatorClimberDown = operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_DOWN);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        initializeSubsystems();
        // Configure the button bindings
        configureButtonBindings();
        ControllerQueue.getInstance().addController(operatorJoystick);
    }

    private void initializeSubsystems() {
        initializeDrive();
    }

    private void initializeDrive() {
        switch (RobotConstants.get().robot()) {
            case ROBOT_KITBOT:
            case ROBOT_COMP:
                drive = new Drive(new DriveIOSparkMAX());
                break;
            case ROBOT_SIM:
                drive = new Drive(new DriveIOSim());
                break;
        }
    }

    private void configureSubsystems() {
        configureDrive();
    }

    private void configureDrive() {
        drive.setDefaultCommand(new DriveWithJoystick(drive, () -> -driverJoystick.getLeftY(), () -> -driverJoystick.getRightX()));
//        drive.setDefaultCommand(new DriveWithTank(drive, () -> -driverJoystick.getLeftY(), () -> -driverJoystick.getRightY()));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
        configureSubsystems();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoCommand;
    }
}
