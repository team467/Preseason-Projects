package frc.robot.subsystems.drive;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import java.util.ArrayList;
import java.util.List;
import lib.io.gyro.GyroIO;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private static final SimpleMotorFeedforward driveFF = RobotConstants.get().moduleDriveFF()
      .getFeedforward(); // Faster to type and shorter to read
  private static final PIDController[] turnFB = new PIDController[4];
  private final ModuleIO[] moduleIOs = new ModuleIO[4];
  private final ModuleIO.ModuleIOInputs[] moduleIOInputs = new ModuleIO.ModuleIOInputs[]{
      new ModuleIO.ModuleIOInputs(), new ModuleIO.ModuleIOInputs(), new ModuleIO.ModuleIOInputs(),
      new ModuleIO.ModuleIOInputs()};
  private final GyroIO gyroIO;
  private final GyroIO.GyroIOInputs gyroIOInputs = new GyroIO.GyroIOInputs();

  private double angle = 0;

  private ChassisSpeeds setpoint = new ChassisSpeeds();

  private final SwerveDriveOdometry odometry;

  private double[] lastModulePositions = new double[]{0.0, 0.0, 0.0, 0.0};

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

    for (int i = 0; i < 4; i++) {
      turnFB[i] = RobotConstants.get().moduleTurnFB().getPIDController();
      turnFB[i].enableContinuousInput(-Math.PI, Math.PI);
    }

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = new SwerveModulePosition(
          moduleIOInputs[i].drivePosition * (RobotConstants.get().moduleWheelDiameter() / 2),
          new Rotation2d(moduleIOInputs[i].turnPositionAbsolute));
    }

    if (gyroIOInputs.connected) {
      odometry = new SwerveDriveOdometry(RobotConstants.get().kinematics(),
          Rotation2d.fromDegrees(gyroIOInputs.angle), modulePositions);
    } else {
      odometry = new SwerveDriveOdometry(RobotConstants.get().kinematics(), new Rotation2d(angle),
          modulePositions);
    }
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

    // Update angle measurements
    Rotation2d[] turnPositions = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      turnPositions[i] =
          new Rotation2d(moduleIOInputs[i].turnPositionAbsolute);
    }

    if (DriverStation.isDisabled()) {
      // Disable output while disabled
      for (int i = 0; i < 4; i++) {
        moduleIOs[i].setTurnVoltage(0.0);
        moduleIOs[i].setDriveVoltage(0.0);
      }
    } else {
      // In normal mode, run the controllers for turning and driving based on the current
      // setpoint
      SwerveModuleState[] setpointStates =
          RobotConstants.get().kinematics().toSwerveModuleStates(setpoint);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,
          RobotConstants.get().maxLinearSpeed());

      // If stationary, go to last state
      boolean isStationary =
          Math.abs(setpoint.vxMetersPerSecond) < 1e-3
              && Math.abs(setpoint.vyMetersPerSecond) < 1e-3
              && Math.abs(setpoint.omegaRadiansPerSecond) < 1e-3;

      SwerveModuleState[] setpointStatesOptimized =
          new SwerveModuleState[]{null, null, null, null};
      for (int i = 0; i < 4; i++) {
        // Run turn controller
        setpointStatesOptimized[i] =
            SwerveModuleState.optimize(setpointStates[i], turnPositions[i]);
        if (isStationary) {
          moduleIOs[i].setTurnVoltage(0.0);
        } else {
          moduleIOs[i].setTurnVoltage(
              turnFB[i].calculate(turnPositions[i].getRadians(),
                  setpointStatesOptimized[i].angle.getRadians()));
        }

        // Update velocity based on turn error
        setpointStatesOptimized[i].speedMetersPerSecond *=
            Math.cos(turnFB[i].getPositionError());

        // Run drive controller
        double velocityRadPerSec =
            setpointStatesOptimized[i].speedMetersPerSecond / (
                RobotConstants.get().moduleWheelDiameter() / 2);
        moduleIOs[i].setDriveVoltage(
            driveFF.calculate(velocityRadPerSec));

        // Log individual setpoints
        Logger.getInstance().recordOutput(
            "SwerveDriveSetpoints/" + Integer.toString(i),
            velocityRadPerSec);
        Logger.getInstance().recordOutput(
            "SwerveTurnSetpoints/" + Integer.toString(i),
            setpointStatesOptimized[i].angle.getRadians());
      }

      // Log all module setpoints
      logModuleStates("SwerveModuleStates/Setpoints", setpointStates);
      logModuleStates("SwerveModuleStates/SetpointsOptimized",
          setpointStatesOptimized);
    }

    SwerveModuleState[] measuredStates =
        new SwerveModuleState[]{null, null, null, null};

    for (int i = 0; i < 4; i++) {
      measuredStates[i] = new SwerveModuleState(
          moduleIOInputs[i].driveVelocity * (RobotConstants.get().moduleWheelDiameter() / 2),
          turnPositions[i]);
    }

    // Update odometry
    SwerveModulePosition[] measuredPositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      measuredPositions[i] = new SwerveModulePosition(
          moduleIOInputs[i].drivePosition * (RobotConstants.get().moduleWheelDiameter() / 2),
          turnPositions[i]);
    }
    if (gyroIOInputs.connected) {
      odometry.update(Rotation2d.fromDegrees(gyroIOInputs.angle), measuredPositions);
    } else {
      angle += RobotConstants.get().kinematics()
          .toChassisSpeeds(measuredStates).omegaRadiansPerSecond;
      odometry.update(new Rotation2d(angle), measuredPositions);
    }
    // Log measured states
    logModuleStates("SwerveModuleStates/Measured", measuredStates);

    // Log odometry pose
    Logger.getInstance().recordOutput("Odometry", getPose());
  }

  /**
   * Log robot swerve module states for AdvantageScope (thanks Mechanical Advantage!)
   *
   * @param key    The name of the field to record. It will be stored under "/RealOutputs" or
   *               "/ReplayOutputs"
   * @param states The states of the wheels.
   */
  private void logModuleStates(String key, SwerveModuleState[] states) {
    List<Double> dataArray = new ArrayList<Double>();
    for (int i = 0; i < 4; i++) {
      dataArray.add(states[i].angle.getRadians());
      dataArray.add(states[i].speedMetersPerSecond);
    }
    Logger.getInstance().recordOutput(key,
        dataArray.stream().mapToDouble(Double::doubleValue).toArray());
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
   * Drive the robot based on given velocities on the x and y-axis.
   *
   * @param x             Velocity of the robot in the x direction.
   * @param y             Velocity of the robot in the y direction.
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the given speeds are field relative.
   */
  public void chassisDrive(double x, double y, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getPose().getRotation()));
    } else {
      runVelocity(new ChassisSpeeds(x, y, rot));
    }
  }
}

