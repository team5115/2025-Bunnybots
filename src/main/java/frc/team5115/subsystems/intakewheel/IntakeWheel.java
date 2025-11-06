package frc.team5115.subsystems.intakewheel;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
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

    public Command stop() {
        return setSpeed(0);
    }

    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }

    public Command intakeIf(BooleanSupplier supplier) {
        return Commands.run(
                () -> {
                    if (supplier.getAsBoolean()) {
                        // stall means we back it up a little and try again
                        if (inputs.currentAmps > 25.0) {
                            io.setPercent(-0.22);
                        } else {
                            io.setPercent(Constants.INTAKE_SPEED);
                        }
                    } else {
                        io.setPercent(0);
                    }
                },
                this);
    }
}
