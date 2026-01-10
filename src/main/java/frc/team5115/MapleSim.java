package frc.team5115;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.team5115.Constants.SwerveConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.littletonrobotics.junction.Logger;

public class MapleSim {

    private static SwerveDriveSimulation swerveSim;

    public static SimulatedArena getInstance() {
        return SimulatedArena.getInstance();
    }

    public static void setupArena() {
        SimulatedArena.getInstance().addGamePiece(new CrescendoNoteOnField(new Translation2d(3, 3)));
    }

    public static void simPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();

        Logger.recordOutput("FieldSimulation/RobotPosition", swerveSim.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Note", SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
    }

    public static DriveTrainSimulationConfig getDriveSimConfig() {
        return DriveTrainSimulationConfig.Default()
                // Specify gyro type (for realistic gyro drifting and error simulation)
                .withGyro(() -> new GyroSimulation(0, 0))
                // Specify swerve module (for realistic swerve dynamics)
                .withSwerveModule(
                        new SwerveModuleSimulationConfig(
                                DCMotor.getNEO(1), // Drive motor is a NEO
                                DCMotor.getNeo550(1), // Steer motor is a NEO 550
                                5.007, // Drive motor gear ratio.
                                12, // Steer motor gear ratio.
                                Volts.of(0.1), // Drive friction voltage.
                                Volts.of(0.1), // Steer friction voltage
                                Inches.of(3), // Wheel radius
                                KilogramSquareMeters.of(0.01), // Steer MOI
                                1.2)) // Wheel COF
                // Configures the track length and track width (spacing between swerve modules)
                .withTrackLengthTrackWidth(
                        Inches.of(SwerveConstants.TRACK_WIDTH_X), Inches.of(SwerveConstants.TRACK_WIDTH_Y))
                // Configures the bumper size (dimensions of the robot bumper)
                .withBumperSize(Inches.of(30), Inches.of(30)); // TODO: correct numbers
    }

    public static void initInstance() {
        swerveSim =
                new SwerveDriveSimulation(
                        // Specify Configuration
                        MapleSim.getDriveSimConfig(),
                        // Specify starting pose
                        new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveSim);
    }

    public static SwerveDriveSimulation getSwerveSim() {
        return swerveSim;
    }
}
