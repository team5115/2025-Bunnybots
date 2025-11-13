package frc.team5115.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public Rotation2d armAngle = new Rotation2d();
        public double armVelocityRPM = 0;
        public double currentAmps = 0;
        public double appliedVolts = 0;

        public boolean luniteDetected = true;
        public boolean luniteDetected2 = true;
        public boolean luniteDetected3 = true;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setArmVoltage(double volts) {}

    public default void getSparks(ArrayList<SparkMax> sparks) {}
}
