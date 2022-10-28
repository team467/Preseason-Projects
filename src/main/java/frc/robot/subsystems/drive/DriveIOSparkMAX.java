package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.robot.RobotConstants;

public class DriveIOSparkMAX implements DriveIO {
    private final CANSparkMax leftMaster;
    private final CANSparkMax rightMaster;
    private final RelativeEncoder leftEncoder;
    private final CANSparkMax leftSlave;
    private final CANSparkMax rightSlave;
    private final RelativeEncoder rightEncoder;

    private final ADIS16470_IMU gyro;

    public DriveIOSparkMAX() {
        leftMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftSlave = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMaster = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightSlave = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();

        leftMaster.setInverted(false);
        rightMaster.setInverted(false);

        leftSlave.setInverted(false);
        rightSlave.setInverted(false);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        // Convert rotations to meters (2pi*radius)
        leftEncoder.setPositionConversionFactor(RobotConstants.get().driveWheelDiameter() * Math.PI * RobotConstants.get().driveGearRatio().getRotationsPerInput());
        rightEncoder.setPositionConversionFactor(RobotConstants.get().driveWheelDiameter() * Math.PI * RobotConstants.get().driveGearRatio().getRotationsPerInput());

        // Convert rotations per minute to meters per second (2pi*radius/60)
        leftEncoder.setVelocityConversionFactor(RobotConstants.get().driveWheelDiameter() * Math.PI * RobotConstants.get().driveGearRatio().getRotationsPerInput() / 60);
        rightEncoder.setVelocityConversionFactor(RobotConstants.get().driveWheelDiameter() * Math.PI * RobotConstants.get().driveGearRatio().getRotationsPerInput() / 60);

        leftMaster.enableVoltageCompensation(12);
        rightMaster.enableVoltageCompensation(12);
        leftSlave.enableVoltageCompensation(12);
        rightSlave.enableVoltageCompensation(12);

        gyro = new ADIS16470_IMU();
        gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);
        gyro.calibrate();
        gyro.reset();
    }

    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftPosition = leftEncoder.getPosition();
        inputs.leftVelocity = leftEncoder.getVelocity();
        inputs.leftAppliedVolts = leftMaster.getAppliedOutput();
        inputs.leftCurrent = new double[]{leftMaster.getOutputCurrent(), leftSlave.getOutputCurrent()};
        inputs.leftTemp = new double[]{leftMaster.getMotorTemperature(), leftSlave.getMotorTemperature()};

        inputs.rightPosition = rightEncoder.getPosition();
        inputs.rightVelocity = rightEncoder.getVelocity();
        inputs.rightAppliedVolts = rightMaster.getAppliedOutput();
        inputs.rightCurrent = new double[]{rightMaster.getOutputCurrent(), rightSlave.getOutputCurrent()};
        inputs.rightTemp = new double[]{rightMaster.getMotorTemperature(), rightSlave.getMotorTemperature()};

        inputs.angle = gyro.getAngle();
        inputs.rate = gyro.getRate();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(rightVolts);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
}
