package frc.robot.io;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

/**
 * CANcoder implementation of EncoderIO that handles absolute position sensing.
 *
 * OFFSET HANDLING ARCHITECTURE:
 * This class uses a two-part offset system to handle arbitrary mechanism offsets:
 *
 * 1. MagnetOffset (hardware offset): Applied by the CANcoder firmware directly to raw sensor readings.
 *    - MUST be in range [-1, 1] encoder rotations (API limitation - values outside this range are silently modulo'd)
 *    - Applied before any gear ratio conversion
 *    - Measured in encoder rotations
 *
 * 2. extraOffset (software offset): Applied in software after unit conversion.
 *    - Handles the "overflow" when desired offset exceeds MagnetOffset range
 *    - Always a multiple of 2π (full rotations) when possible
 *    - Measured in mechanism units
 *
 * MATHEMATICAL RELATIONSHIPS:
 * - Encoder rotations to mechanism units: mech_rad = (enc_rot * 2π) / encoderRatio
 * - Total offset = (MagnetOffset * 2π / encoderRatio) + extraOffset (both in mechanism unitns)
 * - When setting an offset, we try to maximize use of MagnetOffset (for accuracy) while keeping extraOffset as multiples of 2π
 */
public class EncoderIOCANcoder extends EncoderIO {
    private CANcoder encoder;
    private CANcoderConfiguration config = new CANcoderConfiguration();
    private boolean configChanged = true;

    /**
     * Gear ratio between encoder and mechanism.
     * Defined as: (encoder radians) / (mechanism units)
     * Example: If encoder does 10 radians for 1 mechanism unit, encoderRatio = 10
     */
    private double encoderRatio = 1;

    /**
     * Tracks the offset applied via setOffset() for simulation purposes.
     * When setOffset() calls encoder.setPosition(), it creates an internal offset in the CANcoder.
     * We need to account for this when setting simulated positions via setRawPosition().
     *
     * The relationship is: reportedPosition = rawPosition - simOffset (for non-inverted)
     * So to achieve a target reportedPosition: rawPosition = target + simOffset
     */
    private double simOffset = 0;

    private CANcoderSimState sim;

    private int id;
    private CANBus canBus;

    private boolean disconnected = false;

    public EncoderIOCANcoder(int id, CANBus canBus, String name, String logPath) {
        super(name, logPath);
        encoder = new CANcoder(id, canBus);
        sim = encoder.getSimState();
        this.id = id;
        this.canBus = canBus;
    }

    public EncoderIOCANcoder(int id, String canBus, String name, String logPath) {
        this(id, new CANBus(canBus), name, logPath);
    }

    public EncoderIOCANcoder(int id, String name, String logPath) {
        this(id, new CANBus(), name, logPath);
    }

    public int getId() {
        return id;
    }

    public double getRatio() {
        return encoderRatio;
    }

    public CANBus getCANBus() {
        return canBus;
    }

    @Override
    public void update() {
        if (configChanged) {
            configChanged = false;
            encoder.getConfigurator().apply(config);
        }

        inputs.connected = disconnected ? false : encoder.isConnected();

        // Position calculation pipeline:
        // 1. CANcoder returns position in encoder rotations (with offset already applied)
        // 2. Convert to encoder radians: enc_rot * 2π
        // 3. Apply gear ratio to get mechanism units: (enc_rot * 2π) / encoderRatio
        inputs.positionRad = Units.rotationsToRadians(encoder.getPosition().getValueAsDouble()) / encoderRatio;
        inputs.velocityRadPerSec =
                Units.rotationsToRadians(encoder.getVelocity().getValueAsDouble()) / encoderRatio;

        // Update fault inputs
        inputs.badMagnetFault = encoder.getFault_BadMagnet().getValue();
        inputs.hardwareFault = encoder.getFault_Hardware().getValue();

        // Update alerts using the base class method (this checks all fault conditions and updates dashboard alerts)
        super.update();
    }

    /**
     * Sets the gear ratio between encoder and mechanism.
     * @param ratio The ratio defined as (encoder radians)/(mechanism units)
     */
    @Override
    public void setGearRatio(double ratio) {
        encoderRatio = ratio;
        configChanged = true;
    }

    /**
     * Sets the offset of the encoder in mechanism radians.
     *
     * @param mechOffset Desired offset in mechanism radians
     */
    @Override
    public void setOffset(double mechOffset) {
        // Convert mechanism offset to encoder rotations
        double rotOffset = Units.radiansToRotations(mechOffset * encoderRatio);

        encoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble() - rotOffset);

        // Track offset for simulation - see simOffset field documentation
        simOffset = rotOffset;
    }

    @Override
    public void setPosition(double mechPos) {
        double actualPos = Units.radiansToRotations(mechPos * encoderRatio);

        encoder.setPosition(actualPos);

        // Direct position set clears offset tracking
        simOffset = 0;
    }

    /**
     * Sets whether the encoder is inverted.
     * @param inverted If true, positive rotation becomes clockwise
     */
    @Override
    public void setInverted(boolean inverted) {
        config.MagnetSensor.SensorDirection =
                inverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        configChanged = true;
    }

    /**
     * Sets the simulated mechanism position.
     * Reverses the calculation from update() to determine what encoder position produces the desired mechanism position.
     *
     * <p>Phoenix 6 simulation note: The SimState API ignores device invert settings.
     * Inversion is applied when reading via getPosition(), not when setting raw position.
     * So for inverted encoders, we must negate the raw position we set.
     *
     * <p>Offset handling: When setOffset() is called, it uses encoder.setPosition() which creates
     * an internal offset. The relationship becomes: reportedPosition = rawPosition - simOffset
     * (for non-inverted). We account for this by adding simOffset to our target.
     *
     * @param position Desired mechanism position in mechanism radians
     */
    @Override
    public void setMechPosition(double position) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechPosition on " + getName(), AlertType.kWarning);
            return;
        }

        // Convert mechanism position to encoder rotations (this is what getPosition() should return)
        double targetPosition = Units.radiansToRotations(position * encoderRatio);

        // Account for offset: reportedPosition = rawPosition - simOffset
        // So: rawPosition = targetPosition + simOffset
        double rawPosition = targetPosition + simOffset;

        // Apply inversion: SimState ignores invert settings, but getPosition() applies them.
        // For inverted (Clockwise_Positive): getPosition() = -rawPosition - simOffset
        // So we need: rawPosition = -(targetPosition + simOffset)
        if (config.MagnetSensor.SensorDirection.equals(SensorDirectionValue.Clockwise_Positive)) {
            rawPosition = -rawPosition;
        }

        sim.setRawPosition(rawPosition);
    }

    /**
     * Sets the simulated mechanism velocity.
     *
     * <p>Phoenix 6 simulation note: The SimState API ignores device invert settings.
     * Inversion is applied when reading via getVelocity(), not when setting velocity.
     * So for inverted encoders, we must negate the velocity we set.
     *
     * @param velocity Mechanism velocity in mechanism radians per second
     */
    @Override
    public void setMechVelocity(double velocity) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechVelocity on " + getName(), AlertType.kWarning);
            return;
        }

        // Convert mechanism velocity to encoder velocity (rotations per second)
        double encoderVel = Units.radiansToRotations(velocity * encoderRatio);

        // Apply inversion: SimState ignores invert settings, but getVelocity() applies them.
        if (config.MagnetSensor.SensorDirection.equals(SensorDirectionValue.Clockwise_Positive)) {
            encoderVel = -encoderVel;
        }

        sim.setVelocity(encoderVel);
    }

    @Override
    public void disconnect() {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method disconnect on " + getName(), AlertType.kWarning);
            return;
        }
        disconnected = true;
    }
}
