package frc.team5115;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.catcher.Catcher;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.intakewheel.IntakeWheel;
import frc.team5115.subsystems.outtake.Outtake;

public class DriverController {
    private final CommandXboxController joyDrive;
    private final CommandXboxController joyManip;

    private final Drivetrain drivetrain;

    private boolean robotRelative = false;
    private boolean slowMode = false;

    public DriverController(int port, Drivetrain drivetrain) {
        joyDrive = new CommandXboxController(port);
        joyManip = null;

        this.drivetrain = drivetrain;
    }

    public DriverController(int drivePort, int manipPort, Drivetrain drivetrain) {
        joyDrive = new CommandXboxController(drivePort);
        joyManip = new CommandXboxController(manipPort);

        this.drivetrain = drivetrain;
    }

    private Command offsetGyro() {
        return Commands.runOnce(() -> drivetrain.offsetGyro(), drivetrain).ignoringDisable(true);
    }

    public boolean isConnected() {
        return joyDrive.isConnected() && (joyManip == null || joyManip.isConnected());
    }

    public void configureButtonBindings(
            Arm arm, Outtake outtake, IntakeWheel intakeWheel, Catcher catcher) {
        // drive control
        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> robotRelative,
                        () -> slowMode,
                        () -> -joyDrive.getLeftY(),
                        () -> -joyDrive.getLeftX(),
                        () -> -joyDrive.getRightX()));
        if (joyManip == null) {
            configureSingleMode(arm, outtake, intakeWheel, catcher);
        } else {
            configureDualMode(arm, outtake, intakeWheel, catcher);
        }
    }

    private Command setRobotRelative(boolean state) {
        return Commands.runOnce(() -> robotRelative = state);
    }

    private Command setSlowMode(boolean state) {
        return Commands.runOnce(() -> slowMode = state);
    }

    public boolean getRobotRelative() {
        return robotRelative;
    }

    public boolean getSlowMode() {
        return slowMode;
    }

    private void configureSingleMode(
            Arm arm, Outtake outtake, IntakeWheel intakeWheel, Catcher catcher) {

        // joyDrive.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        joyDrive.leftBumper().onTrue(setRobotRelative(true)).onFalse(setRobotRelative(false));
        joyDrive.rightBumper().onTrue(setSlowMode(true)).onFalse(setSlowMode(false));
        joyDrive.start().onTrue(offsetGyro());

        arm.filterTimeElapsed()
                .and(() -> !outtake.isLockOverride())
                .onTrue(DriveCommands.xferLunite(outtake, arm, intakeWheel));

        joyDrive
                .b()
                .and(() -> !outtake.getLocked())
                .onTrue(outtake.extend())
                .onFalse(outtake.retract());
        joyDrive.back().onTrue(DriveCommands.vomit(arm, intakeWheel));
        joyDrive
                .rightTrigger()
                .onTrue(outtake.setLockOverride(true))
                .onFalse(outtake.setLockOverride(false));

        joyDrive.x().onTrue(DriveCommands.xferLunite(outtake, arm, intakeWheel));

        joyDrive.povUp().onTrue(catcher.extendNet());
        joyDrive.povDown().onTrue(catcher.retractNet());

        joyDrive.a().onTrue(DriveCommands.intake(arm, intakeWheel));

        joyDrive.y().onTrue(DriveCommands.stow(arm, intakeWheel));
    }

    private void configureDualMode(
            Arm arm, Outtake outtake, IntakeWheel intakeWheel, Catcher catcher) {

        joyDrive.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        joyDrive.leftBumper().onTrue(setRobotRelative(true)).onFalse(setRobotRelative(false));
        joyDrive.rightBumper().onTrue(setSlowMode(true)).onFalse(setSlowMode(false));
        joyDrive.start().onTrue(offsetGyro());

        arm.filterTimeElapsed()
                .and(() -> !outtake.isLockOverride())
                .onTrue(DriveCommands.xferLunite(outtake, arm, intakeWheel));

        joyManip.x().onTrue(DriveCommands.xferLunite(outtake, arm, intakeWheel));

        joyManip
                .b()
                .and(() -> !outtake.getLocked())
                .onTrue(outtake.extend())
                .onFalse(outtake.retract());
        joyManip.back().onTrue(DriveCommands.vomit(arm, intakeWheel));
        joyManip
                .rightTrigger()
                .onTrue(outtake.setLockOverride(true))
                .onFalse(outtake.setLockOverride(false));

        joyManip.povUp().onTrue(catcher.extendNet());
        joyManip.povDown().onTrue(catcher.retractNet());

        joyManip.a().onTrue(DriveCommands.intake(arm, intakeWheel));

        joyManip.y().onTrue(DriveCommands.stow(arm, intakeWheel));
    }
}
