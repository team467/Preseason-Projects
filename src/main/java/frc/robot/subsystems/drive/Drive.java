package frc.robot.subsystems.drive;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private SimpleMotorFeedforward driveFF = RobotConstants.get().driveFF().getFeedforward();
    private PIDController turnFB = RobotConstants.get().turnFB().getPIDController();
    private final ModuleIO[] moduleIOs = new ModuleIO[4];
    private final ModuleIO.ModuleIOInputs[] moduleIOInputs = new ModuleIO.ModuleIOInputs[]{new ModuleIO.ModuleIOInputs(), new ModuleIO.ModuleIOInputs(), new ModuleIO.ModuleIOInputs(), new ModuleIO.ModuleIOInputs()};
    private final GyroIO gyroIO;
    private final GyroIO.GyroIOInputs gyroIOInputs = new GyroIO.GyroIOInputs();

    private ChassisSpeeds setpoint = new ChassisSpeeds();

    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(RobotConstants.get().swerveKinematics(), Rotation2d.fromDegrees(gyroIOInputs.angle));

    /**
     * Configures the drive subsystem
     *
     * @param gyroIO Gyro IO
     * @param flIO   Front Left Module IO
     * @param frIO   Front Right Module IO
     * @param blIO   Back Left Module IO
     * @param brIO   Back Right Module IO
     */
    public Drive(GyroIO gyroIO, ModuleIO flIO, ModuleIO frIO, ModuleIO blIO, ModuleIO brIO) {
        super();
        this.gyroIO = gyroIO;
        moduleIOs[0] = flIO;
        moduleIOs[1] = frIO;
        moduleIOs[2] = blIO;
        moduleIOs[3] = brIO;

        turnFB.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        // Update inputs for IOs
        gyroIO.updateInputs(gyroIOInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroIOInputs);
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].updateInputs(moduleIOInputs[i]);
            Logger.getInstance().processInputs("Drive/Module" + i, moduleIOInputs[i]);
            Logger.getInstance().recordOutput("TurnPositions/" + i, moduleIOInputs[i].turnPosition);
        }

        // Convert current setpoint to module states
        SwerveModuleState[] moduleStates = RobotConstants.get().swerveKinematics().toSwerveModuleStates(setpoint);
        // Limit max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, RobotConstants.get().maxLinearSpeed());
        for (int i = 0; i < 4; i++) {
            // We should not turn more than 90 degrees
            SwerveModuleState optimizedState = SwerveModuleState.optimize(moduleStates[i], new Rotation2d(moduleIOInputs[i].turnPosition));
            moduleIOs[i].setTurnVoltage(turnFB.calculate(moduleIOInputs[i].turnPosition, optimizedState.angle.getRadians()));

            // Meters per second to radians per second
            double driveVelocity = optimizedState.speedMetersPerSecond / (RobotConstants.get().wheelDiameter() / 2);
            moduleIOs[i].setDriveVoltage(driveFF.calculate(driveVelocity));

            Logger.getInstance().recordOutput("DriveSetpoints/" + i, driveVelocity);
            Logger.getInstance().recordOutput("TurnSetpoints/" + i, optimizedState.angle.getRadians());
        }

        odometry.update(Rotation2d.fromDegrees(gyroIOInputs.angle), moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3]);
        Logger.getInstance().recordOutput("Odometry", new double[]{odometry.getPoseMeters().getTranslation().getX(), odometry.getPoseMeters().getTranslation().getY(), odometry.getPoseMeters().getRotation().getRadians()});
    }

    /**
     * Set the setpoint of the velocity controller to the given chassis speeds.
     *
     * @param speeds The desired speeds of the robot.
     */
    public void runVelocity(ChassisSpeeds speeds) {
        setpoint = speeds;
    }

    /**
     * Stops the robot.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Get the current pose of the robot.
     *
     * @return The current pose of the robot.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Drive the robot based on given speeds on the x and y-axis.
     *
     * @param x             Speed of the robot in the x direction.
     * @param y             Speed of the robot in the y direction.
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the given speeds are field relative.
     */
    public void speedDrive(double x, double y, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, Rotation2d.fromDegrees(gyroIOInputs.angle)));
        } else {
            runVelocity(new ChassisSpeeds(x, y, rot));
        }
    }
}

