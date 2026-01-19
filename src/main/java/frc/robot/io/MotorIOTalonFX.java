package frc.robot.io;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

// Make a CTRE TalonFX-backed implementation of MotorIO.
// Units used:
// - Mechanism position in radians (rad) for arms/flywheels, meters (m) for elevators
// - Speeds in rad/s or m/s
// - Voltages in volts, currents in amps
public class MotorIOTalonFX extends MotorIO {
    private TalonFX motor;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private boolean configChanged = true;
    private boolean brakeOnNeutral = false;
    private boolean disabled = false;

    private TalonFXSimState sim;

    // Control objects (one per control mode)
    private CoastOut coast = new CoastOut();
    private StaticBrake brake = new StaticBrake();

    private DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private VoltageOut voltage = new VoltageOut(0);
    private TorqueCurrentFOC torqueCurrent = new TorqueCurrentFOC(0);
    private MotionMagicTorqueCurrentFOC motionMagicTorqueCurrent = new MotionMagicTorqueCurrentFOC(0);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private MotionMagicVelocityVoltage magicVelocityVoltage = new MotionMagicVelocityVoltage(0);
    private MotionMagicVelocityTorqueCurrentFOC magicVelocityTorqueCurrent = new MotionMagicVelocityTorqueCurrentFOC(0);
    private PositionVoltage positionVoltage = new PositionVoltage(0);
    private PositionTorqueCurrentFOC positionCurrent = new PositionTorqueCurrentFOC(0);
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private VelocityTorqueCurrentFOC velocityCurrent = new VelocityTorqueCurrentFOC(0);
    private Follower follow = new Follower(0, MotorAlignmentValue.Aligned);

    private enum ControlType {
        COAST,
        BRAKE,
        NEUTRAL,
        DUTY_CYCLE,
        VOLTAGE,
        TORQUE_CURRENT,
        POS_VOLTAGE,
        POS_CURRENT,
        VEL_VOLTAGE,
        VEL_CURRENT,
        MM_POS_VOLTAGE,
        MM_POS_CURRENT,
        MM_VEL_VOLTAGE,
        MM_VEL_CURRENT,
        FOLLOW
    }

    private ControlType currentControl = ControlType.NEUTRAL;

    // Feedforward lambda
    private Supplier<Double> feedforward;

    // Software offset in mechanism radians
    private double extraOffset;

    // Encoder connected to this motor
    private EncoderIOCANcoder connectedEncoder;

    private double minLimit = -Double.MAX_VALUE;
    private double maxLimit = Double.MAX_VALUE;

    // Make a TalonFX on the given CAN bus
    public MotorIOTalonFX(int id, CANBus canBus, String name, String logPath) {
        super(name, logPath);
        motor = new TalonFX(id, canBus);
        sim = motor.getSimState();
    }

    // Make a TalonFX on a named CAN bus (e.g., "rio", "canivore")
    public MotorIOTalonFX(int id, String canBus, String name, String logPath) {
        this(id, new CANBus(canBus), name, logPath);
    }

    // Make a TalonFX on the default CAN bus
    public MotorIOTalonFX(int id, String name, String logPath) {
        this(id, new CANBus(), name, logPath);
    }

    @Override
    public void update() {
        if (configChanged) {
            configChanged = false;
            motor.getConfigurator().apply(config);
        }

        if (disabled) {
            currentControl = ControlType.NEUTRAL;
        }

        // Update all input values from the motor signals
        inputs.connected = motor.isConnected();

        // Convert rotations to radians for mechanism units
        inputs.position = Units.rotationsToRadians(motor.getPosition().getValueAsDouble()) - extraOffset;
        inputs.velocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.accel = Units.rotationsToRadians(motor.getAcceleration().getValueAsDouble());

        inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyVoltage = motor.getSupplyVoltage().getValueAsDouble();
        inputs.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
        inputs.torqueCurrent = motor.getTorqueCurrent().getValueAsDouble();

        inputs.controlMode = currentControl.name();

        double setpoint =
                Units.rotationsToRadians(motor.getClosedLoopReference().getValueAsDouble());
        switch (currentControl) {
            case COAST, BRAKE, NEUTRAL, FOLLOW, VOLTAGE, DUTY_CYCLE, TORQUE_CURRENT:
                inputs.setpoint = 0;
                break;
            case VEL_CURRENT, VEL_VOLTAGE, MM_VEL_CURRENT, MM_VEL_VOLTAGE:
                inputs.setpoint = setpoint;
                break;
            default:
                inputs.setpoint = setpoint - extraOffset;
                break;
        }

        inputs.setpointVelocity =
                Units.rotationsToRadians(motor.getClosedLoopReferenceSlope().getValueAsDouble());

        inputs.error = Units.rotationsToRadians(motor.getClosedLoopError().getValueAsDouble());
        inputs.feedforward = motor.getClosedLoopFeedForward().getValueAsDouble();
        inputs.derivOutput = motor.getClosedLoopDerivativeOutput().getValueAsDouble();
        inputs.intOutput = motor.getClosedLoopIntegratedOutput().getValueAsDouble();
        inputs.propOutput = motor.getClosedLoopProportionalOutput().getValueAsDouble();

        inputs.temp = motor.getDeviceTemp().getValueAsDouble();
        inputs.dutyCycle = motor.getDutyCycle().getValueAsDouble();

        inputs.hardwareFault = motor.getFault_Hardware().getValue();
        inputs.tempFault = motor.getFault_DeviceTemp().getValue();
        inputs.forwardLimitFault = motor.getFault_ForwardHardLimit().getValue()
                || motor.getFault_ForwardSoftLimit().getValue();
        inputs.reverseLimitFault = motor.getFault_ReverseHardLimit().getValue()
                || motor.getFault_ReverseSoftLimit().getValue();

        inputs.rawRotorPosition =
                Units.rotationsToRadians(motor.getRotorPosition().getValueAsDouble()
                        / (config.Feedback.RotorToSensorRatio * config.Feedback.SensorToMechanismRatio));

        if (connectedEncoder != null) {
            inputs.encoderDiff = inputs.position - connectedEncoder.getInputs().positionRad;
        }

        // Update alerts using the base class method (this checks all fault conditions and updates dashboard alerts)
        super.update();

        // Current feedforward given by FF lambda
        double currentFeedforward = feedforward == null ? 0 : feedforward.get();

        // Set motor control
        switch (currentControl) {
            case BRAKE:
                motor.setControl(brake);
                break;
            case COAST:
                motor.setControl(coast);
                break;
            case NEUTRAL:
                if (brakeOnNeutral) {
                    motor.setControl(brake);
                } else {
                    motor.setControl(coast);
                }
                break;
            case VOLTAGE:
                motor.setControl(voltage);
                break;
            case DUTY_CYCLE:
                motor.setControl(dutyCycle);
                break;
            case TORQUE_CURRENT:
                motor.setControl(torqueCurrent);
                break;
            case POS_CURRENT:
                motor.setControl(positionCurrent.withFeedForward(currentFeedforward));
                break;
            case POS_VOLTAGE:
                motor.setControl(positionVoltage.withFeedForward(currentFeedforward));
                break;
            case VEL_CURRENT:
                motor.setControl(velocityCurrent.withFeedForward(currentFeedforward));
                break;
            case VEL_VOLTAGE:
                motor.setControl(velocityVoltage.withFeedForward(currentFeedforward));
                break;
            case MM_POS_CURRENT:
                motor.setControl(motionMagicTorqueCurrent.withFeedForward(currentFeedforward));
                break;
            case MM_POS_VOLTAGE:
                motor.setControl(motionMagicVoltage.withFeedForward(currentFeedforward));
                break;
            case MM_VEL_CURRENT:
                motor.setControl(magicVelocityTorqueCurrent.withFeedForward(currentFeedforward));
                break;
            case MM_VEL_VOLTAGE:
                motor.setControl(magicVelocityVoltage.withFeedForward(currentFeedforward));
                break;
            case FOLLOW:
                motor.setControl(follow);
                break;
        }
    }

    @Override
    public void brake() {
        currentControl = ControlType.BRAKE;
    }

    @Override
    public void coast() {
        currentControl = ControlType.COAST;
    }

    @Override
    public void neutral() {
        currentControl = ControlType.NEUTRAL;
    }

    // Tell the motor how fast to spin (percent, -1 = full reverse, 1 = full forward)
    @Override
    public void setDutyCycle(double value) {
        dutyCycle.withOutput(value);
        currentControl = ControlType.DUTY_CYCLE;
    }

    // Tell the motor what voltage to apply (volts). Similar to setDutyCycle but in volts.
    @Override
    public void setVoltage(double volts) {
        voltage.withOutput(volts);
        currentControl = ControlType.VOLTAGE;
    }

    // Tell the motor the torque-producing current to use (amps). Helpful to ignore battery sag and back-EMF.
    @Override
    public void setTorqueCurrent(double current) {
        torqueCurrent.withOutput(current);
        currentControl = ControlType.TORQUE_CURRENT;
    }

    // Tell the motor to go to a target position using Motion Magic with current control (radians)
    @Override
    public void setGoalWithCurrentMagic(double position, Supplier<Double> feedforward) {
        position = MathUtil.clamp(position, minLimit, maxLimit);
        motionMagicTorqueCurrent.withPosition(Units.radiansToRotations(position + extraOffset));
        currentControl = ControlType.MM_POS_CURRENT;
        this.feedforward = feedforward;
    }

    // Tell the motor to go to a target position using Motion Magic with voltage control (radians)
    @Override
    public void setGoalWithVoltageMagic(double position) {
        position = MathUtil.clamp(position, minLimit, maxLimit);
        motionMagicVoltage.withPosition(Units.radiansToRotations(position + extraOffset));
        currentControl = ControlType.MM_POS_VOLTAGE;
    }

    // Tell the motor to reach a target speed using Motion Magic with current control (rad/s)
    @Override
    public void setVelocityWithCurrentMagic(double velocity) {
        magicVelocityTorqueCurrent.withVelocity(Units.radiansToRotations(velocity));
        currentControl = ControlType.MM_VEL_CURRENT;
    }

    // Tell the motor to reach a target speed using Motion Magic with voltage control (rad/s)
    @Override
    public void setVelocityWithVoltageMagic(double velocity) {
        magicVelocityVoltage.withVelocity(Units.radiansToRotations(velocity));
        currentControl = ControlType.MM_VEL_VOLTAGE;
    }

    // Tell the motor to go to a target position using current control (radians)
    @Override
    public void setGoalWithCurrent(double position) {
        position = MathUtil.clamp(position, minLimit, maxLimit);
        positionCurrent.withPosition(Units.radiansToRotations(position + extraOffset));
        currentControl = ControlType.POS_CURRENT;
    }

    // Tell the motor to go to a target position using voltage control (radians)
    @Override
    public void setGoalWithVoltage(double position) {
        position = MathUtil.clamp(position, minLimit, maxLimit);
        positionVoltage.withPosition(Units.radiansToRotations(position + extraOffset));
        currentControl = ControlType.POS_VOLTAGE;
    }

    // Tell the motor to reach a target speed using current control (rad/s)
    @Override
    public void setVelocityWithCurrent(double velocity) {
        velocityCurrent.withVelocity(Units.radiansToRotations(velocity));
        currentControl = ControlType.VEL_CURRENT;
    }

    // Tell the motor to reach a target speed using voltage control (rad/s)
    @Override
    public void setVelocityWithVoltage(double velocity) {
        velocityVoltage.withVelocity(Units.radiansToRotations(velocity));
        currentControl = ControlType.VEL_VOLTAGE;
    }

    // Make this motor follow another motor with the given CAN ID (invert if needed).
    // Note: Only CTRE motors on the same CAN bus can be followed.
    @Override
    public void follow(int motorId, boolean invert) {
        follow.withLeaderID(motorId)
                .withMotorAlignment(invert ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned);
        currentControl = ControlType.FOLLOW;
    }

    // Tell the motor which direction is forward (true = invert)
    @Override
    public void setInverted(boolean inverted) {
        InvertedValue newInverted =
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        if (newInverted != config.MotorOutput.Inverted) {
            config.MotorOutput.Inverted = newInverted;
            configChanged = true;
        }
    }

    // Tell the motor what to do when stopped: brake (hold) or coast (freewheel)
    @Override
    public void setBraking(boolean brake) {
        brakeOnNeutral = brake;
    }

    // Make PID and feedforward values active (converting from rotations-based to radians-based where needed)
    @Override
    public void setkP(double kP) {
        double newkP = Units.rotationsToRadians(kP);
        if (newkP != config.Slot0.kP) {
            config.Slot0.kP = newkP;
            configChanged = true;
        }
    }

    @Override
    public void setkI(double kI) {
        double newkI = Units.rotationsToRadians(kI);
        if (newkI != config.Slot0.kI) {
            config.Slot0.kI = newkI;
            configChanged = true;
        }
    }

    @Override
    public void setkD(double kD) {
        double newkD = Units.rotationsToRadians(kD);
        if (newkD != config.Slot0.kD) {
            config.Slot0.kD = newkD;
            configChanged = true;
        }
    }

    @Override
    public void setkS(double kS) {
        if (kS != config.Slot0.kS) {
            config.Slot0.kS = kS;
            configChanged = true;
        }
    }

    @Override
    public void setkG(double kG) {
        if (kG != config.Slot0.kG) {
            config.Slot0.kG = kG;
            configChanged = true;
        }
    }

    @Override
    public void setkV(double kV) {
        double newkV = Units.rotationsToRadians(kV);
        if (newkV != config.Slot0.kV) {
            config.Slot0.kV = newkV;
            configChanged = true;
        }
    }

    @Override
    public void setkA(double kA) {
        double newkA = Units.rotationsToRadians(kA);
        if (newkA != config.Slot0.kA) {
            config.Slot0.kA = newkA;
            configChanged = true;
        }
    }

    // Modifies gains for unit scaling
    @Override
    public void setGains(Slot0Configs gains) {
        setkP(gains.kP);
        setkI(gains.kI);
        setkD(gains.kD);
        setkG(gains.kG);
        setkS(gains.kS);
        setkV(gains.kV);
        setkA(gains.kA);
        setFeedforwardType(gains.GravityType);
        setStaticFeedforwardType(gains.StaticFeedforwardSign);
    }

    @Override
    public void setMaxVelocity(double maxVelocity) {
        double newMaxVelocity = Units.radiansToRotations(maxVelocity);
        if (newMaxVelocity != config.MotionMagic.MotionMagicCruiseVelocity) {
            config.MotionMagic.MotionMagicCruiseVelocity = newMaxVelocity;
            configChanged = true;
        }
    }

    @Override
    public void setMaxAccel(double maxAccel) {
        double newMaxAccel = Units.radiansToRotations(maxAccel);
        if (newMaxAccel != config.MotionMagic.MotionMagicAcceleration) {
            config.MotionMagic.MotionMagicAcceleration = newMaxAccel;
            configChanged = true;
        }
    }

    @Override
    public void setMaxJerk(double maxJerk) {
        double newMaxJerk = Units.radiansToRotations(maxJerk);
        if (newMaxJerk != config.MotionMagic.MotionMagicJerk) {
            config.MotionMagic.MotionMagicJerk = newMaxJerk;
            configChanged = true;
        }
    }

    // Make continuous wrap enabled for mechanisms that can spin > 360° (like swerve azimuth)
    @Override
    public void setContinuousWrap(boolean continuousWrap) {
        if (continuousWrap != config.ClosedLoopGeneral.ContinuousWrap) {
            config.ClosedLoopGeneral.ContinuousWrap = continuousWrap;
            configChanged = true;
        }
    }

    // Tell the controller which gravity model to use (Arm_Cosine or Elevator_Static)
    @Override
    public void setFeedforwardType(GravityTypeValue type) {
        if (type != config.Slot0.GravityType) {
            config.Slot0.GravityType = type;
            configChanged = true;
        }
    }

    // Tell the controller which gravity model to use (Arm_Cosine or Elevator_Static)
    @Override
    public void setStaticFeedforwardType(StaticFeedforwardSignValue type) {
        if (type != config.Slot0.StaticFeedforwardSign) {
            config.Slot0.StaticFeedforwardSign = type;
            configChanged = true;
        }
    }

    // Tell the motor to use a remote encoder with gear ratios:
    // - motorToSensorRatio: motor rotations to sensor rotations (unitless)
    // - fuse: Whether to use the internal rotor along with the CANcoder. Always set to true, unless there are issues
    // with the reported position teleporting even after accounting for gear ratio and inversion, in which case it
    // should be false
    // Only use ONE of connectEncoder OR setGearRatio for a motor, not both.
    // Currently only supports CANcoders.
    @Override
    public void connectEncoder(EncoderIO encoder, double motorToSensorRatio, boolean fuse) {
        if (encoder instanceof EncoderIOCANcoder cancoder) {
            config.Feedback.FeedbackRemoteSensorID = cancoder.getId();
            config.Feedback.FeedbackSensorSource =
                    fuse ? FeedbackSensorSourceValue.FusedCANcoder : FeedbackSensorSourceValue.RemoteCANcoder;
            config.Feedback.RotorToSensorRatio = motorToSensorRatio;
            config.Feedback.SensorToMechanismRatio = cancoder.getRatio();
            connectedEncoder = cancoder;
            configChanged = true;
        } else {
            Alerts.create(
                    "TalonFX " + getName() + " doesn't support feedback sources other than CANcoders",
                    AlertType.kError);
        }
    }

    // Tell the motor to use its internal sensor with a gear ratio to the mechanism
    @Override
    public void setGearRatio(double motorToMechanismRatio) {
        config.Feedback.RotorToSensorRatio = 1;
        config.Feedback.SensorToMechanismRatio = motorToMechanismRatio;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configChanged = true;
    }

    // Use after connectEncoder/setGearRatio. Sets the mechanism offset.
    @Override
    public void setOffset(double offset) {
        if (config.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.FusedCANcoder
                || config.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RemoteCANcoder) {
            connectedEncoder.setOffset(offset);
            extraOffset = connectedEncoder.getExtraOffset();
        } else if (config.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RotorSensor) {
            double ratio = config.Feedback.SensorToMechanismRatio;

            // Convert mechanism offset to mech rotations
            double rotOffset = Units.radiansToRotations(offset);

            // Wrap to [-0.5, 0.5] range to find the fractional rotation part
            double remOffset = rotOffset - Math.round(rotOffset);

            double rotorOffset = remOffset * ratio;

            if (Math.abs(rotorOffset) <= 1) {
                config.Feedback.FeedbackRotorOffset = rotorOffset;

                // extraOffset handles the integer rotations (converted back to mechanism radians)
                // This will be a multiple of 2π, preserving periodicity
                extraOffset = Units.rotationsToRadians(rotOffset - remOffset);
            } else {
                config.Feedback.FeedbackRotorOffset = 0;
                extraOffset = offset;

                // Warn because non-2π multiples in extraOffset break gravity compensation assumptions
                Alerts.create(
                        "extraOffset is not a multiple of 2pi--if " + getName()
                                + " is used in an arm mechanism, kG will not account for gravity correctly",
                        AlertType.kWarning);
            }
        } else {
            Alerts.create("Invalid sensor source for TalonFX " + getName(), AlertType.kError);
        }
    }

    // Current limits:
    // - StatorCurrentLimit: limit on torque-producing current (amps)
    // - SupplyCurrentLimit: limit on battery current draw (amps)
    // - If current > SupplyCurrentLowerLimit for SupplyCurrentLowerTime seconds, clamp to SupplyCurrentLowerLimit
    @Override
    public void setStatorCurrentLimit(double statorCurrentLimit) {
        if (statorCurrentLimit != config.CurrentLimits.StatorCurrentLimit) {
            config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
            configChanged = true;
        }
    }

    @Override
    public void setSupplyCurrentLimit(double supplyCurrentLimit) {
        if (supplyCurrentLimit != config.CurrentLimits.SupplyCurrentLimit) {
            config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
            configChanged = true;
        }
    }

    @Override
    public void setSupplyCurrentLowerLimit(double supplyCurrentLowerLimit) {
        if (supplyCurrentLowerLimit != config.CurrentLimits.SupplyCurrentLowerLimit) {
            config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLowerLimit;
            configChanged = true;
        }
    }

    @Override
    public void setSupplyCurrentLowerTime(double supplyCurrentLowerTime) {
        if (supplyCurrentLowerTime != config.CurrentLimits.SupplyCurrentLowerTime) {
            config.CurrentLimits.SupplyCurrentLowerTime = supplyCurrentLowerTime;
            configChanged = true;
        }
    }

    @Override
    public void setLimits(double min, double max) {
        minLimit = min;
        maxLimit = max;
        double newForwardThreshold = Units.radiansToRotations(max + extraOffset);
        double newReverseThreshold = Units.radiansToRotations(min + extraOffset);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = newForwardThreshold;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = newReverseThreshold;
        configChanged = true;
    }

    // We apply invert after adding offset because invert is applied before offset in the position reading code
    @Override
    public void setMechPosition(double position) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechPosition on " + getName(), AlertType.kWarning);
            return;
        }
        double rotorPos = Units.radiansToRotations(
                (position + extraOffset) * config.Feedback.RotorToSensorRatio * config.Feedback.SensorToMechanismRatio);
        if (config.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RotorSensor) {
            rotorPos += config.Feedback.FeedbackRotorOffset;
        }
        rotorPos = config.MotorOutput.Inverted.equals(InvertedValue.Clockwise_Positive) ? -rotorPos : rotorPos;
        sim.setRawRotorPosition(rotorPos);
    }

    @Override
    public void setMechVelocity(double velocity) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechVelocity on " + getName(), AlertType.kWarning);
            return;
        }
        double rotorVel = Units.radiansToRotations(
                velocity * config.Feedback.RotorToSensorRatio * config.Feedback.SensorToMechanismRatio);
        rotorVel = config.MotorOutput.Inverted.equals(InvertedValue.Clockwise_Positive) ? -rotorVel : rotorVel;
        sim.setRotorVelocity(rotorVel);
    }

    @Override
    public void clearStickyFaults() {
        motor.clearStickyFaults();
    }

    @Override
    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }
}
