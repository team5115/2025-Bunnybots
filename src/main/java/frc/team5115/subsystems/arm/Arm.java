package frc.team5115.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5115.Constants;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final ArmFeedforward feedforward;
    private final PIDController pid;
    private final double ks;
    public final Trigger sensorTrigger;
    private final Timer timer;

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
        pid.setSetpoint(75.0);

        sensorTrigger = new Trigger(() -> (inputs.luniteDetected == true));
        timer = new Timer();

        sensorTrigger
                .onTrue(Commands.runOnce(() -> timer.restart()))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    timer.stop();
                                    timer.reset();
                                }));
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

    public Command setAngle(Rotation2d setpoint) {
        return Commands.runOnce(() -> pid.setSetpoint(setpoint.getDegrees()));
    }

    public Command goToAngle(Rotation2d setpoint, double timeout) {
        return setAngle(setpoint).andThen(waitForSetpoint(timeout));
    }

    public Command stow() {
        return setAngle(Rotation2d.fromDegrees(Constants.ARM_STOW_ANGLE_DEG));
    }

    public Command deploy() {
        return setAngle(Rotation2d.fromDegrees(Constants.ARM_DEPLOY_ANGLE_DEG));
    }

    public Command waitForSensorState(boolean state, double timeout) {
        return Commands.waitUntil(() -> inputs.luniteDetected == state).withTimeout(timeout);
    }

    public Trigger sensorTrigger() {
        return new Trigger(() -> inputs.luniteDetected);
    }

    public Trigger filterTimeElapsed() {
        return new Trigger(() -> timer.hasElapsed(Constants.SENSOR_FILTER_TIME));
    }

    public void stop() {
        io.setArmVoltage(0);
    }
}
