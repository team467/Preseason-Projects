package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DriveIO {
    class DriveIOInputs implements LoggableInputs {
        public double leftPosition = 0.0;
        public double leftVelocity = 0.0;
        public double leftAppliedVolts = 0.0;
        public double[] leftCurrent = new double[]{};
        public double[] leftTemp = new double[]{};

        public double rightPosition = 0.0;
        public double rightVelocity = 0.0;
        public double rightAppliedVolts = 0.0;
        public double[] rightCurrent = new double[]{};
        public double[] rightTemp = new double[]{};

        public double angle = 0.0;
        public double rate = 0.0;

        public void toLog(LogTable table) {
            table.put("LeftPosition", leftPosition);
            table.put("LeftVelocity", leftVelocity);
            table.put("LeftAppliedVolts", leftAppliedVolts);
            table.put("LeftCurrent", leftCurrent);
            table.put("LeftTemp", leftTemp);

            table.put("RightPosition", rightPosition);
            table.put("RightVelocity", rightVelocity);
            table.put("RightAppliedVolts", rightAppliedVolts);
            table.put("RightCurrent", rightCurrent);
            table.put("RightTemp", rightTemp);

            table.put("Angle", angle);
            table.put("Rate", rate);
        }

        public void fromLog(LogTable table) {
            leftPosition = table.getDouble("LeftPosition", leftPosition);
            leftVelocity = table.getDouble("LeftVelocity", leftVelocity);
            leftAppliedVolts = table.getDouble("LeftAppliedVolts", leftAppliedVolts);
            leftCurrent = table.getDoubleArray("LeftCurrent", leftCurrent);
            leftTemp = table.getDoubleArray("LeftTemp", leftTemp);

            rightPosition = table.getDouble("RightPosition", rightPosition);
            rightVelocity = table.getDouble("RightVelocity", rightVelocity);
            rightAppliedVolts = table.getDouble("RightAppliedVolts", rightAppliedVolts);
            rightCurrent = table.getDoubleArray("RightCurrent", rightCurrent);
            rightTemp = table.getDoubleArray("RightTemp", rightTemp);

            angle = table.getDouble("Angle", angle);
            rate = table.getDouble("Rate", rate);
        }
    }

    void updateInputs(DriveIOInputs inputs);

    void tankDriveVolts(double leftVolts, double rightVolts);
}
