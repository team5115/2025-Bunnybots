package frc.team5115.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.Constants;
import frc.team5115.Constants.SwerveConstants;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.intakewheel.IntakeWheel;
import frc.team5115.subsystems.outtake.Outtake;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {
    private static final double DEADBAND = 0.1;

    private DriveCommands() {}

    /**
     * Field or robot relative drive command using two joysticks (controlling linear and angular
     * velocities).
     */
    public static Command joystickDrive(
            Drivetrain drivetrain,
            BooleanSupplier robotRelative,
            BooleanSupplier slowMode,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Apply deadband
                    double linearMagnitude =
                            MathUtil.applyDeadband(
                                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                    Rotation2d linearDirection;
                    if (xSupplier.getAsDouble() == 0 && ySupplier.getAsDouble() == 0) {
                        linearDirection = new Rotation2d();
                    } else {
                        linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    }
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square values
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

                    // Calcaulate new linear velocity
                    Translation2d linearVelocity =
                            new Pose2d(new Translation2d(), linearDirection)
                                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                    .getTranslation();

                    // Convert to ChassisSpeeds & send command
                    final double multiplier = slowMode.getAsBoolean() ? Constants.SLOW_MODE_SPEED : 1.0;
                    final double vx = linearVelocity.getX() * SwerveConstants.MAX_LINEAR_SPEED * multiplier;
                    final double vy = linearVelocity.getY() * SwerveConstants.MAX_LINEAR_SPEED * multiplier;
                    omega *= SwerveConstants.MAX_ANGULAR_SPEED * multiplier;
                    drivetrain.runVelocity(
                            robotRelative.getAsBoolean()
                                    ? new ChassisSpeeds(vx, vy, omega)
                                    : ChassisSpeeds.fromFieldRelativeSpeeds(
                                            vx, vy, omega, drivetrain.getGyroRotation()));
                },
                drivetrain);
    }

    public static Command xferLunite(Outtake outtake, Arm arm, IntakeWheel intakeWheel) {
        return Commands.sequence(
                outtake.retract(),
                outtake.setLock(true),
                arm.stow(),
                arm.waitForSetpoint(1.5),
                intakeWheel.xfer(),
                arm.waitForSensorState(false, 1),
                Commands.waitSeconds(0.5),
                outtake.setLock(false));
    }

    public static Command intake(Arm arm, IntakeWheel intakeWheel) {
        return Commands.sequence(
                arm.deploy(),
                intakeWheel.intake(),
                arm.waitForSensorState(true, Double.POSITIVE_INFINITY),
                intakeWheel.stop());
    }

    public static Command vomit(Arm arm, IntakeWheel intakeWheel) {
        return Commands.sequence(arm.deploy(), arm.waitForSetpoint(1), intakeWheel.vomit());
    }

    public static Command stow(Arm arm, IntakeWheel intakeWheel) {
        return Commands.parallel(arm.stow(), intakeWheel.intake());
    }
}
