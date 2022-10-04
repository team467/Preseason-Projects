package frc.robot.subsystems.example;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ExampleIO {
    class ExampleIOInputs implements LoggableInputs {
        public double position = 0.0;
        public double velocity = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("Position", position);
            table.put("Velocity", velocity);
        }

        @Override
        public void fromLog(LogTable table) {
            position = table.getDouble("Position", position);
            velocity = table.getDouble("Velocity", velocity);
        }
    }

    void updateInputs(ExampleIOInputs inputs);
}
