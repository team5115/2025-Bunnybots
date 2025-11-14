package frc.team5115;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class MapleSim {
    private final DriveTrainSimulationConfig driveTrainSimulationConfig;

    public static SimulatedArena getInstance() {
        return SimulatedArena.getInstance();
    }

    public static void setupArena() {
        SimulatedArena.getInstance().addGamePiece(new CrescendoNoteOnField(new Translation2d(3, 3)));
    }

    public static void simPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
    }

    public static void setupDriveSim() {
        driveTrainSimulationConfig.DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(COTS.ofPigeon2())
        // Specify swerve module (for realistic swerve dynamics)
        .withSwerveModule(new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                6.12, // Drive motor gear ratio.
                12.8, // Steer motor gear ratio.
                Volts.of(0.1), // Drive friction voltage.
                Volts.of(0.1), // Steer friction voltage
                Inches.of(2), // Wheel radius
                KilogramSquareMeters.of(0.03), // Steer MOI
                1.2)) // Wheel COF
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(30), Inches.of(30));
    }
}
