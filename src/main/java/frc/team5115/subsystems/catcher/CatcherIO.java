package frc.team5115.subsystems.catcher;

import org.littletonrobotics.junction.AutoLog;

public interface CatcherIO {
    @AutoLog
    public static class CatcherIOInputs {
        public boolean state = false;
    }

    public default void extendNet() {}

    public default void retractNet() {}

    public default void updateInputs(CatcherIOInputs inputs) {}
}
