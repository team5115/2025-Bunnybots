package frc.team5115.subsystems.catcher;

import org.littletonrobotics.junction.AutoLog;

public interface CatcherIO {
    @AutoLog
    public static class ClimberIOInputs {}

    public default void extendNet() {}

    public default void retractNet() {}
}
