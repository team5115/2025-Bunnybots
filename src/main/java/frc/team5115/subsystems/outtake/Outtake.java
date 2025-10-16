package frc.team5115.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
    private final OuttakeIO io;
    private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

    public Outtake(OuttakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(this.getName(), inputs);
    }

    private Command extend() {
        return Commands.runOnce(() -> io.setPneumatic(true), this);
    }

    private Command retract() {
        return Commands.runOnce(() -> io.setPneumatic(false), this);
    }
}
