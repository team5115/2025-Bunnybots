package frc.team5115.subsystems.intakewheel;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class IntakeWheel extends SubsystemBase {
    private final IntakeWheelIO io;
    private final IntakeWheelIOInputsAutoLogged inputs = new IntakeWheelIOInputsAutoLogged();

    public IntakeWheel(IntakeWheelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Command setSpeed(double speed) {
        return Commands.runOnce(() -> io.setPercent(speed), this);
    }

    public Command intake() {
        return setSpeed(Constants.INTAKE_SPEED);
    }

    public Command vomit() {
        return setSpeed(-0.22);
    }

    public Command xfer() {
        return setSpeed(-Math.PI/10); // don't worry about it
    }

    public Command stop() {
        return setSpeed(0);
    }

    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }
}
