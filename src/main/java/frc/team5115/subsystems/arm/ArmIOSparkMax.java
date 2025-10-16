package frc.team5115.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team5115.Constants;
import java.util.ArrayList;

public class ArmIOSparkMax implements ArmIO {
    private final SparkMax motor;
    ;
    private final AbsoluteEncoder absoluteEncoder;

    public ArmIOSparkMax() {
        motor = new SparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
        absoluteEncoder = motor.getAbsoluteEncoder();
        final SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(30, 40).idleMode(IdleMode.kCoast);
        motorConfig.absoluteEncoder.positionConversionFactor(180);
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
        if (absoluteEncoder.getPosition() > 160) {
            inputs.armAngle = Rotation2d.fromDegrees(absoluteEncoder.getPosition() - 180.0);
        } else {
            inputs.armAngle = Rotation2d.fromDegrees(absoluteEncoder.getPosition());
        }
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    }
}
