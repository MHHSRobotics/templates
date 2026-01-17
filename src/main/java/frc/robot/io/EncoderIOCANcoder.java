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
     * Software offset applied after unit conversion, in mechanism units.
     * Ideally this is a multiple of 2π to preserve periodicity for arm gravity compensation.
     * Non-multiple values trigger a warning as they may break gravity compensation assumptions.
     */
    private double extraOffset;

    private CANcoderSimState sim;

    private int id;
    private CANBus canBus;

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

    public double getExtraOffset() {
        return extraOffset;
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

        inputs.connected = encoder.isConnected();

        // Position calculation pipeline:
        // 1. CANcoder returns absolute position in encoder rotations (with MagnetOffset already applied)
        // 2. Convert to encoder radians: enc_rot * 2π
        // 3. Apply gear ratio to get mechanism units: (enc_rot * 2π) / encoderRatio
        // 4. Subtract software offset: final_pos = converted_pos - extraOffset
        inputs.positionRad =
                Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble()) / encoderRatio - extraOffset;
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
     * OFFSET SPLITTING ALGORITHM:
     * 1. Convert mechanism offset to encoder rotations: mechOffset * encoderRatio / 2π
     * 2. Split into integer and fractional parts
     * 3. Try to put fractional part into MagnetOffset (if it fits within ±1 rotation)
     * 4. Put the remainder (integer rotations) into extraOffset
     *
     * This ensures maximum use of hardware offset while keeping extraOffset as multiples of 2π when possible.
     *
     * @param mechOffset Desired offset in mechanism radians
     */
    @Override
    public void setOffset(double mechOffset) {
        // Convert mechanism offset to mech rotations
        double rotOffset = Units.radiansToRotations(mechOffset);

        // Wrap to [-0.5, 0.5] range to find the fractional rotation part
        double remOffset = rotOffset - Math.round(rotOffset);

        double magnetOffset = remOffset * encoderRatio;

        if (Math.abs(magnetOffset) <= 1) {
            // IDEAL CASE: We can use MagnetOffset for the fractional part
            // MagnetOffset handles the fractional rotation (in encoder rotations)
            config.MagnetSensor.MagnetOffset = -magnetOffset;

            // extraOffset handles the integer rotations (converted back to mechanism radians)
            // This will be a multiple of 2π, preserving periodicity
            extraOffset = Units.rotationsToRadians(rotOffset - remOffset);
        } else {
            // FALLBACK CASE: If we can't fit in MagnetOffset, put entire offset in software
            config.MagnetSensor.MagnetOffset = 0;
            extraOffset = mechOffset;

            // Warn because non-2π multiples in extraOffset break gravity compensation assumptions
            Alerts.create(
                    "extraOffset is not a multiple of 2pi--if " + getName()
                            + " is used in an arm mechanism, kG will not account for gravity correctly",
                    AlertType.kWarning);
        }
        configChanged = true;
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
     * @param position Desired mechanism position in mech units
     */
    @Override
    public void setMechPosition(double position) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechPosition on " + getName(), AlertType.kWarning);
            return;
        }
        // Reverse the position calculation:
        // 1. Add back the software offset: position + extraOffset
        // 2. Apply gear ratio: (position + extraOffset) * encoderRatio
        // 3. Convert to rotations: ((position + extraOffset) * encoderRatio) / 2π
        // 4. Add hardware offset: final + MagnetOffset
        double encoderPos =
                Units.radiansToRotations((position + extraOffset) * encoderRatio) - config.MagnetSensor.MagnetOffset;

        // Apply inversion if configured
        encoderPos = config.MagnetSensor.SensorDirection.equals(SensorDirectionValue.Clockwise_Positive)
                ? -encoderPos
                : encoderPos;
        sim.setRawPosition(encoderPos);
    }

    /**
     * Sets the simulated mechanism velocity.
     * @param velocity Mechanism velocity in mech units per second
     */
    @Override
    public void setMechVelocity(double velocity) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechVelocity on " + getName(), AlertType.kWarning);
            return;
        }
        // Convert mechanism velocity to encoder velocity
        double encoderVel = Units.radiansToRotations(velocity * encoderRatio);

        // Apply inversion if configured
        encoderVel = config.MagnetSensor.SensorDirection.equals(SensorDirectionValue.Clockwise_Positive)
                ? -encoderVel
                : encoderVel;
        sim.setVelocity(encoderVel);
    }
}
