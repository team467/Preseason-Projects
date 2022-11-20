package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO {
  class ModuleIOInputs implements LoggableInputs {
    public double drivePosition = 0.0;
    public double driveVelocity = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrent = new double[] {};
    public double[] driveTemp = new double[] {};

    public double turnPositionAbsolute = 0.0;
    public double turnPosition = 0.0;
    public double turnVelocity = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrent = new double[] {};
    public double[] turnTemp = new double[] {};

    public void toLog(LogTable table) {
      table.put("DrivePosition", drivePosition);
      table.put("DriveVelocity", driveVelocity);
      table.put("DriveAppliedVolts", driveAppliedVolts);
      table.put("DriveCurrent", driveCurrent);
      table.put("DriveTemp", driveTemp);

      table.put("TurnPositionAbsolute", turnPositionAbsolute);
      table.put("TurnPosition", turnPosition);
      table.put("TurnVelocity", turnVelocity);
      table.put("TurnAppliedVolts", turnAppliedVolts);
      table.put("TurnCurrent", turnCurrent);
      table.put("TurnTemp", turnTemp);
    }

    public void fromLog(LogTable table) {
      drivePosition = table.getDouble("DrivePosition", drivePosition);
      driveVelocity = table.getDouble("DriveVelocity", driveVelocity);
      driveAppliedVolts = table.getDouble("DriveAppliedVolts", driveAppliedVolts);
      driveCurrent = table.getDoubleArray("DriveCurrent", driveCurrent);
      driveTemp = table.getDoubleArray("DriveTemp", driveTemp);

      turnPositionAbsolute = table.getDouble("TurnPositionAbsolute", turnPositionAbsolute);
      turnPosition = table.getDouble("TurnPosition", turnPosition);
      turnVelocity = table.getDouble("TurnVelocity", turnVelocity);
      turnAppliedVolts = table.getDouble("TurnAppliedVolts", turnAppliedVolts);
      turnCurrent = table.getDoubleArray("TurnCurrent", turnCurrent);
      turnTemp = table.getDoubleArray("TurnTemp", turnTemp);
    }
  }

  void updateInputs(ModuleIOInputs inputs);

  void setDriveVoltage(double volts);

  void setTurnVoltage(double volts);
}
