package frc.team5115.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team5115.Constants;
import java.util.ArrayList;

public class ArmIOSparkMax implements ArmIO {
    private final SparkMax motor;

    private final AbsoluteEncoder absoluteEncoder;

    private final DigitalInput sensor;
    private final DigitalInput sensor2;
    private final DigitalInput sensor3;

    public ArmIOSparkMax() {
        sensor = new DigitalInput(Constants.LUNITE_SENSOR);
        sensor2 = new DigitalInput(Constants.LUNITE_SENSOR2);
        sensor3 = new DigitalInput(Constants.LUNITE_SENSOR3);
        motor = new SparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
        absoluteEncoder = motor.getAbsoluteEncoder();
        final SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(30, 40).idleMode(IdleMode.kCoast);
        motorConfig.absoluteEncoder.positionConversionFactor(360);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void getSparks(ArrayList<SparkMax> sparks) {
        sparks.add(motor);
    }
    ;

    @Override
    public void setArmVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armVelocityRPM = absoluteEncoder.getVelocity() * 60.0;
        inputs.armAngle = Rotation2d.fromDegrees(absoluteEncoder.getPosition());
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.luniteDetected = sensor.get();
        inputs.luniteDetected2 = sensor2.get();
        inputs.luniteDetected3 = sensor3.get();
    }
}
