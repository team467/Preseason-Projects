package lib.input;

import edu.wpi.first.wpilibj.XboxController;
import lib.utils.MathUtils;

/**
 * Button and axes assignments for an XInput Controller.
 */
public final class XboxController467 extends XboxController {

    private static final double DEADZONE = 0.1;
    private final double DRIVE_FAST_MAX_SPEED;
    private final double DRIVE_NORMAL_MAX_SPEED;
    private final double DRIVE_NORMAL_TURN_MAX_SPEED;
    private final double DRIVE_SLOW_MAX_SPEED;
    private final double DRIVE_SLOW_TURN_MAX_SPEED;

    /**
     * Construct an instance of a controller.
     * <p>Default values are set here, not recommended to use.</p>
     *
     * @param port port that the xbox controller is plugged into
     */
    public XboxController467(int port) {
        this(port, 1.0, 0.8, 1.0, 0.5, 0.8);
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port               port that the xbox controller is plugged into
     * @param fastMaxSpeed       max speed in turbo mode
     * @param normalMaxSpeed     max speed in normal driving
     * @param normalTurnMaxSpeed max speed when turning
     * @param slowMaxSpeed       max speed in slow mode
     * @param slowTurnMaxSpeed   max speed when turning in slow mode
     */
    public XboxController467(int port, double fastMaxSpeed, double normalMaxSpeed,
                             double normalTurnMaxSpeed, double slowMaxSpeed,
                             double slowTurnMaxSpeed) {
        super(port);
        this.DRIVE_FAST_MAX_SPEED = fastMaxSpeed;
        this.DRIVE_NORMAL_MAX_SPEED = normalMaxSpeed;
        this.DRIVE_NORMAL_TURN_MAX_SPEED = normalTurnMaxSpeed;
        this.DRIVE_SLOW_MAX_SPEED = slowMaxSpeed;
        this.DRIVE_SLOW_TURN_MAX_SPEED = slowTurnMaxSpeed;
    }

    @Override
    public double getLeftX() {
        if (Math.abs(super.getLeftX()) < DEADZONE) {
            return 0;
        }
        return super.getLeftX();
    }

    @Override
    public double getLeftY() {
        if (Math.abs(super.getLeftY()) < DEADZONE) {
            return 0;
        }
        return super.getLeftY();
    }

    @Override
    public double getRightX() {
        if (Math.abs(super.getRightX()) < DEADZONE) {
            return 0;
        }
        return super.getRightX();
    }

    @Override
    public double getRightY() {
        if (Math.abs(super.getRightY()) < DEADZONE) {
            return 0;
        }
        return super.getRightY();
    }

    /**
     * Returns the drive speed, taking the turbo and slow triggers into account.
     *
     * @return adjusted speed
     */
    public double getAdjustedSpeed(final double speed) {
        if (getLeftTriggerAxis() > 0.0) {
            // For some reason, up stick is negative, so we flip it
            return turboFastSpeed(speed);
        } else {
            return turboSlowSpeed(speed);
        }
    }

    /**
     * Multiplies speed by the acceleration determined by the left trigger
     *
     * @param speed regular speed to go
     * @return modified speed to go
     */
    public double turboFastSpeed(final double speed) {
        // Speed multiplied by acceleration determined by left trigger
        return speed
                * MathUtils.weightedAverage(
                DRIVE_NORMAL_MAX_SPEED,
                DRIVE_FAST_MAX_SPEED,
                getLeftTriggerAxis());
    }

    /**
     * Multiplies speed by the acceleration determined by the right trigger
     *
     * @param speed regular speed to go
     * @return modified speed to go
     */
    public double turboSlowSpeed(final double speed) {
        // Speed multiplied by deceleration determined by right trigger
        return speed
                * MathUtils.weightedAverage(
                DRIVE_NORMAL_MAX_SPEED,
                DRIVE_SLOW_MAX_SPEED,
                getRightTriggerAxis());
    }

    /**
     * Gets the adjusted drive speed using y-axis of left stick.
     *
     * @return adjusted speed
     */
    public double getAdjustedDriveSpeed() {
        return -getAdjustedSpeed(getLeftY());
    }

    /**
     * Gets the adjusted turn speed using the x-axis of the right stick.
     * <p>Turning is slower when the robot is driving fast.</p>
     *
     * @return adjusted speed
     */
    public double getAdjustedTurnSpeed() {
        return getAdjustedSpeed(getRightX())
                * MathUtils.weightedAverage(
                DRIVE_NORMAL_TURN_MAX_SPEED,
                DRIVE_SLOW_TURN_MAX_SPEED,
                Math.abs(getAdjustedSpeed(getLeftY())));
    }

    /**
     * Calculate the distance of the left stick from the center position.
     *
     * @return distance of left stick from the center position
     */
    public double getLeftStickDistance() {
        return Math.sqrt((getLeftX() * getLeftX()) + (getLeftY() * getLeftY()));
    }

    /**
     * Calculate the distance of the right stick from the center position.
     *
     * @return distance of right stick from the center position
     */
    public double getRightStickDistance() {
        return Math.sqrt((getRightX() * getRightX()) + (getRightY() * getRightY()));
    }

    //TODO: add javadoc comment

    /**
     * If the stick is in the Y deadzone, return the angle of the X axis. Otherwise, return the angle of
     * the stick
     *
     * @param stickX The X value of the joystick.
     * @param stickY The Y value of the joystick.
     * @return The angle of the joystick
     */
    private double calculateStickAngle(final double stickX, final double stickY) {
        if (stickY == 0.0) {
            // In Y deadzone avoid divide by zero error
            return (stickX > 0.0) ? Math.PI / 2 : (-Math.PI) / 2;
        }

        // Return value in range -PI to PI
        double stickAngle = Math.atan2(stickX, -stickY);

        if (stickY > 0) {
            stickAngle += (stickX > 0) ? Math.PI : -Math.PI;
        }

        return (stickAngle);
    }

    public enum Buttons {
        A(1),
        B(2),
        X(3),
        Y(4),
        BumperLeft(5),
        BumperRight(6),
        Back(7),
        Start(8),
        XBox(9),
        StickLeft(10),
        StickRight(11),
        POVup(12),
        POVright(13),
        POVdown(14),
        POVleft(15);

        public final int value;

        Buttons(int value) {
            this.value = value;
        }
    }

    public enum Axes {
        LeftX(0),
        LeftY(1),
        LeftTrigger(2),
        RightTrigger(3),
        RightX(4),
        RightY(5);

        public final int value;

        Axes(int value) {
            this.value = value;
        }
    }
}
