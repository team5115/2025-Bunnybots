package frc.team5115.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double xyAcceleration = 0;
        public AngularVelocity angularVelocity = RadiansPerSecond.of(0);
    }

    public default void updateInputs(GyroIOInputs inputs) {}
}
