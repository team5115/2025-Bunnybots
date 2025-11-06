package frc.team5115;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants.AutoConstants;
import frc.team5115.Constants.Mode;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.arm.ArmIO;
import frc.team5115.subsystems.arm.ArmIOSim;
import frc.team5115.subsystems.arm.ArmIOSparkMax;
import frc.team5115.subsystems.bling.Bling;
import frc.team5115.subsystems.bling.BlingIO;
import frc.team5115.subsystems.bling.BlingIOReal;
import frc.team5115.subsystems.bling.BlingIOSim;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.drive.GyroIO;
import frc.team5115.subsystems.drive.GyroIONavx;
import frc.team5115.subsystems.drive.ModuleIO;
import frc.team5115.subsystems.drive.ModuleIOSim;
import frc.team5115.subsystems.drive.ModuleIOSparkMax;
import frc.team5115.subsystems.intakewheel.IntakeWheel;
import frc.team5115.subsystems.intakewheel.IntakeWheelIO;
import frc.team5115.subsystems.intakewheel.IntakeWheelIOSim;
import frc.team5115.subsystems.intakewheel.IntakeWheelIOSparkMax;
import frc.team5115.subsystems.outtake.Outtake;
import frc.team5115.subsystems.outtake.OuttakeIO;
import frc.team5115.subsystems.outtake.OuttakeIOReal;
import frc.team5115.subsystems.outtake.OuttakeIOSim;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final GyroIO gyro;
    private final Drivetrain drivetrain;
    private final Bling bling;
    private final Outtake outtake;
    private final Arm arm;
    private final IntakeWheel intakeWheel;

    // Controllers
    private final CommandXboxController joyDrive = new CommandXboxController(0);
    private final CommandXboxController joyManip = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    // Setings
    private boolean robotRelative = false;
    private boolean slowMode = false;
    private boolean hasFaults = true;
    private double faultPrintTimeout = 0;
    private DriverController driverController;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        AutoConstants.precomputeAlignmentPoses(); // Computes robot starting pose with vision
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // final GenericEntry dispenseSpeedEntry =
                //         Shuffleboard.getTab("SmartDashboard").add("Dispenser Speed", 0).getEntry();
                //         () -> dispenseSpeedEntry.getDouble(0.1)
                final PneumaticHub hub = new PneumaticHub(Constants.PNEUMATIC_HUB_ID);
                gyro = new GyroIONavx();
                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIOSparkMax(0),
                                new ModuleIOSparkMax(1),
                                new ModuleIOSparkMax(2),
                                new ModuleIOSparkMax(3));
                bling = new Bling(new BlingIOReal());
                outtake = new Outtake(new OuttakeIOReal(hub));
                arm = new Arm(new ArmIOSparkMax());
                intakeWheel = new IntakeWheel(new IntakeWheelIOSparkMax());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                gyro = new GyroIO() {};
                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                bling = new Bling(new BlingIOSim());
                outtake = new Outtake(new OuttakeIOSim());
                arm = new Arm(new ArmIOSim());
                intakeWheel = new IntakeWheel(new IntakeWheelIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                gyro = new GyroIO() {};
                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                bling = new Bling(new BlingIO() {});
                outtake = new Outtake(new OuttakeIO() {});
                arm = new Arm(new ArmIO() {});
                intakeWheel = new IntakeWheel(new IntakeWheelIO() {});
                break;
        }

        // Register auto commands for pathplanner
        registerCommands(drivetrain);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        autoChooser.addOption(
                "Drive Spin SysId (Quasistatic Forward)",
                drivetrain.sysIdSpinQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive Spin SysId (Quasistatic Reverse)",
                drivetrain.sysIdSpinQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive Spin SysId (Dynamic Forward)",
                drivetrain.sysIdSpinDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive Spin SysId (Dynamic Reverse)",
                drivetrain.sysIdSpinDynamic(SysIdRoutine.Direction.kReverse));

        autoChooser.addOption("Drive All SysIds", drivetrain.driveAllSysIds());

        driverController = new DriverController(0, drivetrain);
        driverController.configureButtonBindings(arm, outtake, intakeWheel);
    }

    public void robotPeriodic() {
        if (Constants.currentMode == Mode.REAL) {
            if (faultPrintTimeout <= 0) {
                final var faults =
                        RobotFaults.fromSubsystems(
                                drivetrain, joyDrive.isConnected() && joyManip.isConnected());
                hasFaults = faults.hasFaults();
                if (hasFaults) {
                    System.err.println(faults.toString());
                }
                faultPrintTimeout = 50;
            }
            faultPrintTimeout -= 1;
            Logger.recordOutput("HasFaults", hasFaults);
            Logger.recordOutput("ClearForMatch", !hasFaults);
        }
    }

    /**
     * Register commands for pathplanner to use in autos
     *
     * @param drivetrain
     * @param vision
     * @param elevator
     * @param dispenser
     * @param intake
     * @param dealgaefacationinator5000
     * @param climber
     */
    public static void registerCommands(Drivetrain drivetrain) {
        // Register commands for pathplanner

        System.out.println("Registered Commands");
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private Command offsetGyro() {
        return Commands.runOnce(() -> drivetrain.offsetGyro(), drivetrain).ignoringDisable(true);
    }

    public void teleopInit() {
        drivetrain.setTeleopCurrentLimit();
        arm.sensorTrigger
                .onTrue(outtake.setLock(true))
                .onFalse(Commands.sequence(Commands.waitSeconds(1), outtake.setLock(false)));
        // drivetrain.offsetGyro(Rotation2d.fromDegrees(-90));
    }

    public void autoInit() {
        drivetrain.setAutoCurrentLimit();
        // Offset gyro to zero
        drivetrain.offsetGyro();
        // Then offset by 180 degrees
        drivetrain.offsetGyro(Rotation2d.k180deg);
    }

    public void disabledPeriodic() {}
}
