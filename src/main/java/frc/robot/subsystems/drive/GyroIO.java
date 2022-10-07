package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO {
    class GyroIOInputs implements LoggableInputs {
        public boolean connected = false;
        public double angle = 0.0;
        public double rate = 0.0;

        public void toLog(LogTable table) {
            table.put("Connected", connected);
            table.put("Angle", angle);
            table.put("Rate", rate);
        }

        public void fromLog(LogTable table) {
            connected = table.getBoolean("Connected", connected);
            angle = table.getDouble("Angle", angle);
            rate = table.getDouble("Rate", rate);
        }
    }

    default void updateInputs(GyroIOInputs inputs) {
        inputs.connected = false;
        inputs.angle = 0.0;
        inputs.rate = 0.0;
    }
}
