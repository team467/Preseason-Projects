package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.ByteLogReplay;
import org.littletonrobotics.junction.io.LogSocketServer;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logger logger = Logger.getInstance();
        setUseTiming(RobotConstants.get().mode() != Constants.Mode.REPLAY);

        // Setup metadata
        logger.recordMetadata("Robot", RobotConstants.get().robot().toString());
        logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                logger.recordMetadata("GitDirty", "UNKNOWN");
                break;
        }

        // Setup logging io
        switch (RobotConstants.get().mode()) {
            case REAL:
                String folder = RobotConstants.get().logFolder();
                if (folder != null) {
                    logger.addDataReceiver(new ByteLogReceiver(folder));
                }
                logger.addDataReceiver(new LogSocketServer(5822));
                LoggedSystemStats.getInstance().setPowerDistributionConfig(20, PowerDistribution.ModuleType.kCTRE);
                break;
            case SIM:
                logger.addDataReceiver(new LogSocketServer(5822));
                break;

            case REPLAY:
                String path = ByteLogReplay.promptForPath();
                logger.setReplaySource(new ByteLogReplay(path));
                logger.addDataReceiver(new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim")));
                break;
        }

        // Start the logger
        logger.start();

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

//        new Trigger(this::isEnabled)
//                .negate()
//                .debounce(3)
//                .whenActive(new InstantCommand());
    }


    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Log scheduled commands
        Logger.getInstance().recordOutput("ActiveCommands/Scheduler", NetworkTableInstance.getDefault().getEntry("/liveWindow/Ungrouped/Scheduler/Names").getStringArray(new String[]{}));
    }


    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }


    @Override
    public void disabledPeriodic() {
    }


    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }


    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }


    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }


    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }


    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
