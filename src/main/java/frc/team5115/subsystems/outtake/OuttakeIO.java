package frc.team5115.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

    @AutoLog
    public static class OuttakeIOInputs {
        boolean state;
    }

    public default void updateInputs(OuttakeIOInputs inputs) {}

    public default void setPneumatic(boolean extend) {}
}
