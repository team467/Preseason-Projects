package lib.io.digitalinput;

import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.LogTable;

public class DigitalInputIO {
  public static class DigitalInputIOInputs {
    /** The value of the digital input */
    public boolean value = false;

    public void toLog(LogTable table) {
      table.put("Value", value);
    }

    public void fromLog(LogTable table) {
      value = table.getBoolean("Value", value);
    }
  }

  private final DigitalInput digitalInput;

  public DigitalInputIO(int channel) {
    this.digitalInput = new DigitalInput(channel);
  }

  /**
   * Update the inputs of the digital input
   *
   * @param inputs The inputs to update
   */
  public void updateInputs(DigitalInputIOInputs inputs) {
    inputs.value = !digitalInput.get();
  }
}
