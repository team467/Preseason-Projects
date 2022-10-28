package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

// I hate North-East-down so much
public class Drive extends SubsystemBase {
    private final DriveIO io;
    private final DriveIO.DriveIOInputs inputs = new DriveIO.DriveIOInputs();
    private final DifferentialDriveOdometry odometry;

    private final DifferentialDriveKinematics kinematics = RobotConstants.get().driveKinematics();
    private final SimpleMotorFeedforward driveFF = RobotConstants.get().driveFF().getFeedforward();

    public Drive(DriveIO io) {
        super();
        this.io = io;
        this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(inputs.angle));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive", inputs);
        odometry.update(Rotation2d.fromDegrees(inputs.angle), inputs.leftPosition, inputs.rightPosition);
        Logger.getInstance().recordOutput("Odometry", new double[] {odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), odometry.getPoseMeters().getRotation().getRadians()});
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(pose, Rotation2d.fromDegrees(inputs.angle));
    }

    /**
     * Drive the robot using a ChassisSpeed object
     *
     * @param speeds The chassis speeds to drive at
     */
    public void chassisDrive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        tankDriveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    /**
     * Drive the robot using an Arcade Drive
     *
     * @param speed        speed along the x axis [-1.0..1.0], positive is forwards
     * @param rotation     rotation along the z axis [-1.0..1.0], positive is counterclockwise
     * @param squareInputs If set, decreases the input sensitivity at low speeds.
     */
    public void arcadeDriveSpeed(double speed, double rotation, boolean squareInputs) {
        speed = MathUtil.applyDeadband(speed, 0.02);
        rotation = MathUtil.applyDeadband(rotation, 0.02);

        DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(speed, -rotation, squareInputs);

        double maxVel = RobotConstants.get().driveMaxLinearVelocity();
        tankDriveVelocity(speeds.left * maxVel, speeds.right * maxVel);
    }

    /**
     * Drive the robot using a Curvature Drive
     *
     * @param speed       speed along the x axis [-1.0..1.0], positive is forwards
     * @param rotation    rotation along the z axis [-1.0..1.0], positive is counterclockwise
     * @param turnInPlace If set, overrides constant-curvature turning for turn-in-place maneuvers. zRotation will control turning rate instead of curvature.
     */
    public void curvatureDriveSpeed(double speed, double rotation, boolean turnInPlace) {
        speed = MathUtil.applyDeadband(speed, 0.02);
        rotation = MathUtil.applyDeadband(rotation, 0.02);

        DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(speed, rotation, turnInPlace);

        double maxVel = RobotConstants.get().driveMaxLinearVelocity();
        tankDriveVelocity(speeds.left * maxVel, speeds.right * maxVel);
    }

    /**
     * Drives the robot using tank controls.
     *
     * @param leftVolts  The voltage to send to the left side of the drive.
     * @param rightVolts The voltage to send to the right side of the drive.
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        io.tankDriveVolts(leftVolts, rightVolts);
    }

    /**
     * Drives the robot using tank controls.
     *
     * @param leftVelocity  The velocity of the left side of the drive in meters per second.
     * @param rightVelocity The velocity of the right side of the drive in meters per second.
     */
    public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
        double leftVolts = driveFF.calculate(leftVelocity);
        double rightVolts = driveFF.calculate(rightVelocity);
        tankDriveVolts(leftVolts, rightVolts);
    }

    /**
     * Drives the robot using tank controls.
     *
     * @param left         The speed of the left side of the drive.
     * @param right        The speed of the right side of the drive.
     * @param squareInputs If set, decreases the input sensitivity at low speeds.
     */
    public void tankDriveSpeed(double left, double right, boolean squareInputs) {
        left = MathUtil.applyDeadband(left, 0.02);
        right = MathUtil.applyDeadband(right, 0.02);

        DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.tankDriveIK(left, right, squareInputs);

        double maxVel = RobotConstants.get().driveMaxLinearVelocity();
        tankDriveVelocity(speeds.left * maxVel, speeds.right * maxVel);
    }

    /**
     * Stops the robot from driving.
     */
    public void stop() {
        tankDriveVolts(0, 0);
    }


}
