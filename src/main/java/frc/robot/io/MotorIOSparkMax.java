package frc.robot.io;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

// Make a REV SparkMax-backed implementation of MotorIO.
// Uses WPILib PIDController for closed-loop control instead of SparkMax's built-in PID.
// Units used:
// - Mechanism position in radians (rad) for arms/flywheels, meters (m) for elevators
// - Speeds in rad/s or m/s
// - Voltages in volts, currents in amps
public class MotorIOSparkMax extends MotorIO {
    private SparkMax motor;
    private SparkMaxConfig config = new SparkMaxConfig();
    private boolean configChanged = true;
    private boolean disabled = false;
    private boolean brakeOnNeutral = true;

    // Simulation objects
    private SparkMaxSim sim;
    private SparkRelativeEncoderSim encoderSim;
    private boolean inverted = false;

    // WPILib PID controllers for position and velocity control
    private PIDController positionPID = new PIDController(0, 0, 0);
    private PIDController velocityPID = new PIDController(0, 0, 0);

    // Feedforward gains
    private double kS = 0; // Static friction (volts)
    private double kG = 0; // Gravity compensation (volts)
    private double kV = 0; // Velocity feedforward (volts per rad/s)
    private double kA = 0; // Acceleration feedforward (volts per rad/s^2)

    // Gravity type for built-in feedforward (Arm_Cosine or Elevator_Static)
    private GravityTypeValue gravityType = GravityTypeValue.Elevator_Static;

    // Static feedforward sign type (UseVelocitySign or UseClosedLoopSign)
    private StaticFeedforwardSignValue staticFFSign = StaticFeedforwardSignValue.UseVelocitySign;

    // Feedforward lambda (for custom dynamic feedforward)
    private Supplier<Double> feedforward;

    // Gear ratio from motor rotations to mechanism units
    private double gearRatio = 1.0;

    // Software offset in mechanism radians
    private double offset = 0;

    // Connected external encoder (null = use motor's built-in encoder)
    private EncoderIO connectedEncoder;
    private double motorToSensorRatio = 1.0;

    // Soft limits
    private double minLimit = -Double.MAX_VALUE;
    private double maxLimit = Double.MAX_VALUE;

    private enum SparkControlType {
        BRAKE,
        COAST,
        NEUTRAL,
        DUTY_CYCLE,
        VOLTAGE,
        POSITION,
        VELOCITY,
        FOLLOW
    }

    private SparkControlType currentControl = SparkControlType.NEUTRAL;
    private double setpointValue = 0;

    // For acceleration estimation
    private double prevVelocity = 0;
    private double prevVelocitySetpoint = 0;

    // Make a SparkMax with brushless motor (NEO)
    public MotorIOSparkMax(int id, String name, String logPath) {
        this(id, MotorType.kBrushless, name, logPath);
    }

    // Make a SparkMax with specified motor type
    public MotorIOSparkMax(int id, MotorType motorType, String name, String logPath) {
        super(name, logPath);
        motor = new SparkMax(id, motorType);

        // Initialize simulation objects when not running on real hardware
        // Uses a default NEO motor model - actual physics are handled externally (e.g., in subsystem sim)
        if (Constants.currentMode != Mode.REAL) {
            sim = new SparkMaxSim(motor, DCMotor.getNEO(1));
            encoderSim = sim.getRelativeEncoderSim();
        }
    }

    private void applyConfig() {
        if (configChanged) {
            configChanged = false;
            REVLibError error =
                    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            if (error != REVLibError.kOk) {
                Alerts.create("SparkMax " + getName() + " config error: " + error.name(), AlertType.kWarning);
            }
        }
    }

    @Override
    public void update() {
        applyConfig();

        // Update all input values from the motor
        Faults faults = motor.getStickyFaults();

        // SparkMax doesn't have a direct "isConnected" method, check CAN fault
        inputs.connected = !faults.can;

        // Get position/velocity from motor's built-in encoder
        double rawPosition = motor.getEncoder().getPosition(); // rotations
        double rawVelocity = motor.getEncoder().getVelocity(); // RPM
        double motorPosition = Units.rotationsToRadians(rawPosition / gearRatio) - offset;
        double motorVelocity = Units.rotationsToRadians(rawVelocity / 60.0 / gearRatio); // RPM to rad/s

        // Use connected encoder for feedback if available, otherwise use motor encoder
        if (connectedEncoder != null) {
            inputs.position = connectedEncoder.getInputs().positionRad;
            inputs.velocity = connectedEncoder.getInputs().velocityRadPerSec;
            inputs.encoderDiff = motorPosition - inputs.position;
        } else {
            inputs.position = motorPosition;
            inputs.velocity = motorVelocity;
        }

        // Estimate acceleration from velocity change (SparkMax doesn't provide it directly)
        inputs.accel = (inputs.velocity - prevVelocity) / 0.02; // Assuming 20ms loop
        prevVelocity = inputs.velocity;

        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.supplyVoltage = motor.getBusVoltage();
        inputs.supplyCurrent = motor.getOutputCurrent();
        inputs.torqueCurrent = motor.getOutputCurrent(); // SparkMax doesn't distinguish torque vs supply current

        inputs.controlMode = currentControl.name();

        inputs.setpoint = setpointValue;
        inputs.setpointVelocity = 0; // Not available for simple PID
        inputs.error = inputs.setpoint - inputs.position;

        // Calculate current feedforward value
        double currentFeedforward = feedforward == null ? 0 : feedforward.get();
        inputs.feedforward = currentFeedforward;

        switch(currentControl){
            case POSITION:
            inputs.derivOutput = positionPID.getD();
            inputs.intOutput = positionPID.getI();
            inputs.propOutput = positionPID.getP();
            break;
            case VELOCITY:
            inputs.derivOutput = velocityPID.getD();
            inputs.intOutput = velocityPID.getI();
            inputs.propOutput = velocityPID.getP();
            break;
            default:
            inputs.derivOutput = 0;
            inputs.intOutput = 0;
            inputs.propOutput = 0;
            break;
        }

        inputs.temp = motor.getMotorTemperature();
        inputs.dutyCycle = motor.getAppliedOutput();

        inputs.hardwareFault = faults.motorType || faults.gateDriver || faults.escEeprom || faults.firmware;
        inputs.tempFault = faults.temperature;
        inputs.forwardLimitFault = motor.getForwardSoftLimit().isReached();
        inputs.reverseLimitFault = motor.getReverseSoftLimit().isReached();

        inputs.rawRotorPosition = Units.rotationsToRadians(rawPosition);

        // Update alerts using the base class method
        super.update();

        // Apply control
        if (disabled) {
            currentControl = SparkControlType.NEUTRAL;
        }

        switch (currentControl) {
            case BRAKE:
                setIdleMode(IdleMode.kBrake);
                motor.stopMotor();
                break;
            case COAST:
                setIdleMode(IdleMode.kCoast);
                motor.stopMotor();
                break;
            case NEUTRAL:
                setIdleMode(brakeOnNeutral ? IdleMode.kBrake : IdleMode.kCoast);
                motor.stopMotor();
                break;
            case DUTY_CYCLE:
                motor.set(setpointValue);
                break;
            case VOLTAGE:
                motor.setVoltage(setpointValue);
                break;
            case POSITION:
                double posPID = positionPID.calculate(inputs.position, setpointValue);
                // Calculate gravity feedforward based on type
                double gravityFF = switch (gravityType) {
                    case Arm_Cosine -> kG * Math.cos(inputs.position);
                    case Elevator_Static -> kG;
                };
                // Calculate kS sign based on static feedforward sign type
                double posStaticSign = switch (staticFFSign) {
                    case UseVelocitySign -> Math.signum(inputs.velocity);
                    case UseClosedLoopSign -> Math.signum(posPID);
                };
                double posFeedforward = currentFeedforward + kS * posStaticSign + gravityFF;
                motor.setVoltage(posPID + posFeedforward);
                break;
            case VELOCITY:
                double velPID = velocityPID.calculate(inputs.velocity, setpointValue);
                // Calculate desired acceleration from setpoint change
                double desiredAccel = (setpointValue - prevVelocitySetpoint) / 0.02;
                prevVelocitySetpoint = setpointValue;
                // Calculate kS sign based on static feedforward sign type
                double velStaticSign = switch (staticFFSign) {
                    case UseVelocitySign -> Math.signum(setpointValue);
                    case UseClosedLoopSign -> Math.signum(velPID);
                };
                double velFeedforward = currentFeedforward
                        + kS * velStaticSign
                        + kV * setpointValue
                        + kA * desiredAccel;
                motor.setVoltage(velPID + velFeedforward);
                break;
            case FOLLOW:
                // Follow is configured via config, nothing to do here
                break;
        }
    }

    // Track current idle mode to avoid unnecessary config writes
    private IdleMode currentIdleMode = IdleMode.kBrake;

    // Set idle mode only if it changed (to avoid unnecessary config writes)
    private void setIdleMode(IdleMode mode) {
        if (mode != currentIdleMode) {
            currentIdleMode = mode;
            config.idleMode(mode);
            configChanged = true;
        }
    }

    @Override
    public void brake() {
        currentControl = SparkControlType.BRAKE;
    }

    @Override
    public void coast() {
        currentControl = SparkControlType.COAST;
    }

    @Override
    public void neutral() {
        currentControl = SparkControlType.NEUTRAL;
    }

    @Override
    public void setDutyCycle(double value) {
        setpointValue = value;
        currentControl = SparkControlType.DUTY_CYCLE;
    }

    @Override
    public void setVoltage(double volts) {
        setpointValue = volts;
        currentControl = SparkControlType.VOLTAGE;
    }

    @Override
    public void setGoalWithVoltage(double position) {
        // Clamp to soft limits
        setpointValue = MathUtil.clamp(position, minLimit, maxLimit);
        currentControl = SparkControlType.POSITION;
    }

    // Set position goal with custom feedforward supplier
    public void setGoalWithVoltage(double position, Supplier<Double> feedforward) {
        this.feedforward = feedforward;
        setGoalWithVoltage(position);
    }

    @Override
    public void setVelocityWithVoltage(double velocity) {
        // Initialize prevVelocitySetpoint when entering velocity mode to avoid kA spike
        if (currentControl != SparkControlType.VELOCITY) {
            prevVelocitySetpoint = velocity;
        }
        setpointValue = velocity;
        currentControl = SparkControlType.VELOCITY;
    }

    // Set velocity goal with custom feedforward supplier
    public void setVelocityWithVoltage(double velocity, Supplier<Double> feedforward) {
        this.feedforward = feedforward;
        setVelocityWithVoltage(velocity);
    }

    @Override
    public void follow(int motorId, boolean invert) {
        config.follow(motorId, invert);
        configChanged = true;
        currentControl = SparkControlType.FOLLOW;
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        config.inverted(inverted);
        configChanged = true;
    }

    @Override
    public void setBraking(boolean braking) {
        brakeOnNeutral=braking;
    }

    // PID gains - applied to WPILib PIDController (mechanism units: radians or meters)
    @Override
    public void setkP(double kP) {
        positionPID.setP(kP);
        velocityPID.setP(kP);
    }

    @Override
    public void setkI(double kI) {
        positionPID.setI(kI);
        velocityPID.setI(kI);
    }

    @Override
    public void setkD(double kD) {
        positionPID.setD(kD);
        velocityPID.setD(kD);
    }

    @Override
    public void setkS(double kS) {
        this.kS = kS;
    }

    @Override
    public void setkG(double kG) {
        this.kG = kG;
    }

    @Override
    public void setkV(double kV) {
        this.kV = kV;
    }

    @Override
    public void setkA(double kA) {
        this.kA = kA;
    }

    // Tell the controller which gravity model to use (Arm_Cosine or Elevator_Static)
    @Override
    public void setFeedforwardType(GravityTypeValue type) {
        this.gravityType = type;
    }

    // Tell the controller which sign to use for static feedforward (UseVelocitySign or UseClosedLoopSign)
    @Override
    public void setStaticFeedforwardType(StaticFeedforwardSignValue type) {
        this.staticFFSign = type;
    }

    // Set feedforward supplier for custom dynamic feedforward
    public void setFeedforward(Supplier<Double> feedforward) {
        this.feedforward = feedforward;
    }

    @Override
    public void setContinuousWrap(boolean wrap) {
        if (wrap) {
            positionPID.enableContinuousInput(-Math.PI, Math.PI);
        } else {
            positionPID.disableContinuousInput();
        }
    }

    // Tell the motor to use an external encoder for PID feedback
    // motorToSensorRatio: motor rotations per encoder rotation (for tracking, encoder handles its own gear ratio)
    // fuse parameter is ignored for SparkMax (no hardware fusion like TalonFX)
    @Override
    public void connectEncoder(EncoderIO encoder, double motorToSensorRatio, boolean fuse) {
        this.connectedEncoder = encoder;
        this.motorToSensorRatio = motorToSensorRatio;
    }

    @Override
    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }

    @Override
    public void setOffset(double offset) {
        this.offset = offset;
    }

    // SparkMax supports smart current limiting
    @Override
    public void setSupplyCurrentLimit(double supplyCurrentLimit) {
        config.smartCurrentLimit((int) supplyCurrentLimit);
        configChanged = true;
    }

    @Override
    public void setStatorCurrentLimit(double statorCurrentLimit) {
        // SparkMax doesn't distinguish stator vs supply, use secondary current limit
        config.secondaryCurrentLimit(statorCurrentLimit);
        configChanged = true;
    }

    @Override
    public void setLimits(double min, double max) {
        minLimit = min;
        maxLimit = max;
        // Also configure hardware soft limits as backup
        double minRotations = Units.radiansToRotations(min + offset) * gearRatio;
        double maxRotations = Units.radiansToRotations(max + offset) * gearRatio;
        config.softLimit.forwardSoftLimit(maxRotations).forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(minRotations).reverseSoftLimitEnabled(true);
        configChanged = true;
    }

    @Override
    public void clearStickyFaults() {
        motor.clearFaults();
    }

    @Override
    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
        if (disabled) {
            positionPID.reset();
            velocityPID.reset();
        }
    }

    @Override
    public void setMechPosition(double position) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechPosition on " + getName(), AlertType.kWarning);
            return;
        }
        if (encoderSim != null) {
            // Convert mechanism position (radians) to motor rotations
            double motorRotations = Units.radiansToRotations(position + offset) * gearRatio;
            // Apply inversion
            motorRotations = inverted ? -motorRotations : motorRotations;
            encoderSim.setPosition(motorRotations);
        }
    }

    @Override
    public void setMechVelocity(double velocity) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechVelocity on " + getName(), AlertType.kWarning);
            return;
        }
        if (encoderSim != null) {
            // Convert mechanism velocity (rad/s) to motor RPM
            double motorRPM = Units.radiansToRotations(velocity) * gearRatio * 60.0;
            // Apply inversion
            motorRPM = inverted ? -motorRPM : motorRPM;
            encoderSim.setVelocity(motorRPM);
        }
    }
}
