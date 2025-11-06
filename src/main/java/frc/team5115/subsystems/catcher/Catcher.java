package frc.team5115.subsystems.catcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Catcher extends SubsystemBase {
    private final CatcherIO io;

    public Catcher(CatcherIO io) {
        this.io = io;
    }

    public Command extendNet() {
        return Commands.runOnce(io::extendNet, this);
    }

    public Command retractNet() {
        return Commands.runOnce(io::retractNet, this);
    }
}
