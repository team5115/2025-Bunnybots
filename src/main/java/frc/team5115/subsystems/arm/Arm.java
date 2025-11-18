package frc.team5115.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final ArmFeedforward feedforward;
    private final PIDController pid;
    private final double ks;
    private final SysIdRoutine sysId;
    public boolean mSensor = false;
    public Trigger filterTrigger;
    private Position position = Position.STOWED;

    public enum Position {
        DEPLOYED(0),
        STOWED(75);

        public final Rotation2d rotation;

        Position(double angleDeg) {
            rotation = Rotation2d.fromDegrees(angleDeg);
        }
    }

    public Arm(ArmIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                feedforward = new ArmFeedforward(0.3, 0.35, 0.13509, 0.048686);
                pid = new PIDController(0.405, 0.0, 0.0);
                ks = 0.3;
                break;
            case SIM:
                feedforward = new ArmFeedforward(0.0, 0.35, 0.1351, 0.0);
                pid = new PIDController(0.5, 0.0, 0.0);
                ks = 0.3;
                break;
            default:
                feedforward = new ArmFeedforward(0.0, 0.0, 0, 0.0);
                pid = new PIDController(0.0, 0.0, 0.0);
                ks = 0.3;
                break;
        }

        pid.setTolerance(5);
        pid.setSetpoint(position.rotation.getDegrees());

        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> io.setArmVoltage(voltage.magnitude()), null, this));

        filterTrigger =
                new Trigger(() -> getSensorOutput())
                        .debounce(Constants.SENSOR_FILTER_TIME, DebounceType.kBoth);
    }

    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Arm/Setpoint Degrees", pid.getSetpoint());
        Logger.recordOutput("Arm/At Setpoint?", pid.atSetpoint());
        Logger.recordOutput("Arm/mSensor", this.mSensor);
        Logger.recordOutput("Arm/Net Sensor Output", this.getSensorOutput());
        Logger.recordOutput("Arm/Filtered Sensor", this.filterTrigger.getAsBoolean());

        // Update the pids and feedforward
        final double speed = pid.calculate(inputs.armAngle.getDegrees());
        double voltage = feedforward.calculate(inputs.armAngle.getRadians(), speed);
        voltage = MathUtil.clamp(voltage, -10, +10);

        if (Math.abs(voltage) < 2 * ks) {
            voltage = 0;
        }

        io.setArmVoltage(voltage);
    }
    // meow meow meow, meowwww

    public Command waitForSetpoint(double timeout) {
        return Commands.waitUntil(() -> pid.atSetpoint()).withTimeout(timeout);
    }

    public Command setAngle(Position setpoint) {
        return Commands.runOnce(
                () -> {
                    pid.setSetpoint(setpoint.rotation.getDegrees());
                    position = setpoint;
                });
    }

    public Command goToAngle(Position setpoint, double timeout) {
        return setAngle(setpoint).andThen(waitForSetpoint(timeout));
    }

    @AutoLogOutput
    public Position getPosition() {
        return this.position;
    }

    public Command stow() {
        return setAngle(Position.STOWED);
    }

    public Command deploy() {
        return setAngle(Position.DEPLOYED);
    }

    public Command waitForSensorState(boolean state, double timeout) {
        return Commands.waitUntil(() -> sensorFilter().getAsBoolean() == state).withTimeout(timeout);
    }

    public Command setMSensor(boolean mSensor) {
        return Commands.runOnce(() -> this.mSensor = mSensor);
    }

    public Trigger sensorFilter() {
        return filterTrigger;
    }

    public boolean getSensorOutput() {
        return inputs.luniteDetected || mSensor || inputs.luniteDetected2 || inputs.luniteDetected3;
    }

    public void stop() {
        io.setArmVoltage(0);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }
}
