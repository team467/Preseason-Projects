package lib.io.motorcontroller;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SimpleMotorControllerIO {
    class SimpleMotorControllerIOInputs implements LoggableInputs {
        /**
         * The position of the motor based on if a conversion factor was provided
         */
        public double position = 0.0;
        /**
         * The velocity of the motor based on if a conversion factor was provided
         */
        public double velocity = 0.0;
        /**
         * The voltage applied to the motor
         */
        public double appliedVolts = 0.0;
        /**
         * The current drawn by the motor
         */
        public double[] current = new double[]{};
        /**
         * The temperature of the motor
         */
        public double[] temp = new double[]{};

        public void toLog(LogTable table) {
            table.put("Position", position);
            table.put("Velocity", velocity);
            table.put("AppliedVolts", appliedVolts);
            table.put("Current", current);
            table.put("Temp", temp);
        }

        public void fromLog(LogTable table) {
            position = table.getDouble("Position", position);
            velocity = table.getDouble("Velocity", velocity);
            appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
            current = table.getDoubleArray("Current", current);
            temp = table.getDoubleArray("Temp", temp);
        }
    }

    /**
     * Update the inputs of the motor controller
     *
     * @param inputs The inputs to update
     */
    void updateInputs(SimpleMotorControllerIOInputs inputs);

    /**
     * Set the speed of the motor controller
     *
     * @param speed The speed to set the motor controller to
     */
    void setSpeed(double speed);

    /**
     * Set the voltage of the motor controller
     * <p> Note: This is only supported if a conversion factor to radians was provided
     *
     * @param volts The voltage to set the motor controller to
     */
    void setVoltage(double volts);
}
