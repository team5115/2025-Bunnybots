package frc.team5115;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team5115.Constants.AutoConstants.Side;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.drive.Drivetrain;

public class DriverController {
    private final CommandXboxController joyDrive;
    private final CommandXboxController joyManip;

    private final Drivetrain drivetrain;

    private boolean robotRelative = false;
    private boolean slowMode = false;

    public DriverController(
            int port,
            Drivetrain drivetrain) {
        joyDrive = new CommandXboxController(port);
        joyManip = null;

        this.drivetrain = drivetrain;
        
    }

    public DriverController(
            int drivePort,
            int manipPort,
            Drivetrain drivetrain) {
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

    private void configureSingleMode() {}
    private void configureDualMode() {}
        

    public void configureButtonBindings() {
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
            configureSingleMode();
        } else {
            configureDualMode();
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
}
