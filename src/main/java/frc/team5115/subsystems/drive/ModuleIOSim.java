package frc.team5115.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import frc.team5115.Constants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    private final SimulatedMotorController.GenericMotorController driveSim;
    private final SimulatedMotorController.GenericMotorController turnSim;

    private final SwerveModuleSimulation moduleSimulation;

    // private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        driveSim = this.moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(60));
                // LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, 6.75), DCMotor.getNEO(1));
        turnSim = moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(20));
                        // LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.004, 150.0 / 7.0),
                        // DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // driveSim.update(Constants.LOOP_PERIOD_SECS);
        // turnSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().baseUnitMagnitude();
        inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().baseUnitMagnitude();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorSupplyCurrent().baseUnitMagnitude());

        inputs.turnAbsolutePosition =
                new Rotation2d(moduleSimulation.getSteerAbsoluteAngle());
        inputs.turnVelocityRadPerSec = moduleSimulation.getSteerAbsoluteEncoderSpeed().baseUnitMagnitude();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorSupplyCurrent().baseUnitMagnitude());
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.requestVoltage(Volts.of(driveAppliedVolts));
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.requestVoltage(Volts.of(driveAppliedVolts));
    }
}
