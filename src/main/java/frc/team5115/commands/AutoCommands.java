package frc.team5115.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.intakewheel.IntakeWheel;
import frc.team5115.subsystems.outtake.Outtake;

public class AutoCommands {
    private AutoCommands() {}

    /*
     * Commands required for auto:
     *
     * intake
     *      lower arm & start intake
     *      wait for lunite detection
     *      stow arm & stop intake
     *      xfer lunite (reverse intake)
     *
     * score
     *      raise outtake
     *      wait X seconds
     *      lower outtake
     */
    public static Command intake(Arm arm, IntakeWheel intakeWheel, Outtake outtake) {
        return Commands.sequence(arm.deploy(), intakeWheel.intake(), arm.waitForSensorState(true, 3));
    }

    public static Command xfer(Arm arm, IntakeWheel intakeWheel, Outtake outtake) {
        return DriveCommands.xferLunite(outtake, arm, intakeWheel);
    }

    public static Command score(Outtake outtake) {
        return Commands.sequence(outtake.extend(), Commands.waitSeconds(2), outtake.retract());
    }
}
