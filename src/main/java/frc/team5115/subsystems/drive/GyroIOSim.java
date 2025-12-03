package frc.team5115.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        GyroIO.super.updateInputs(inputs);
    }

    @Override // specified by GroIOSim interface
    public Rotation2d getGyroRotation() {
        return this.gyroSimulation.getGyroReading();
    }

    @Override // specified by GroIOSim interface
    public AngularVelocity getGyroAngularVelocity() {
        return this.gyroSimulation.getMeasuredAngularVelocity();
    }
}
