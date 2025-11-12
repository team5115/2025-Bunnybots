package frc.team5115.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
    private final OuttakeIO io;
    private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
    private boolean locked;
    private boolean lockOverride;

    public Outtake(OuttakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(this.getName(), inputs);
        Logger.recordOutput("Outtake/lockOverride", lockOverride);
        Logger.recordOutput("Outtake/locked", locked);
        Logger.recordOutput("Outtake/actually Locked", getLocked());
    }

    public Command extend() {
        return Commands.runOnce(() -> io.setPneumatic(true), this);
    }

    public Command retract() {
        return Commands.runOnce(() -> io.setPneumatic(false), this);
    }

    public Command setLock(boolean lock) {
        return Commands.runOnce(() -> locked = lock);
    }

    public boolean getLocked() {
        return locked && !lockOverride;
    }

    public boolean isLockOverride() {
        return lockOverride;
    }

    public Command setLockOverride(boolean lockOverride) {
        return Commands.runOnce(() -> this.lockOverride = lockOverride);
    }
}
