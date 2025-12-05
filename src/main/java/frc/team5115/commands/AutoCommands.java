package frc.team5115.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.Constants;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.intakewheel.IntakeWheel;
import frc.team5115.subsystems.outtake.Outtake;

public class AutoCommands {
    private AutoCommands() {}

    public static Command intake(Arm arm, IntakeWheel intakeWheel, Outtake outtake) {
        return Commands.sequence(arm.deploy(), intakeWheel.intake(), arm.waitForSensorState(true, 3));
    }

    public static Command xfer(Arm arm, IntakeWheel intakeWheel, Outtake outtake) {
        return Commands.sequence(
                outtake.retract(),
                arm.stow(),
                Commands.waitSeconds(0.5),
                arm.waitForSetpoint(1.5),
                intakeWheel.xfer(),
                arm.waitForSensorState(false, 1.0),
                Commands.waitSeconds(Constants.EXTRA_XFER_TIME),
                intakeWheel.stop());
    }

    public static Command score(Arm arm, Outtake outtake) {
        return Commands.sequence(
                arm.safeStow(),
                Commands.waitSeconds(0.3),
                arm.waitForSafeToOuttake(0.5),
                outtake.extend(),
                Commands.waitSeconds(1),
                outtake.retract());
    }
}
