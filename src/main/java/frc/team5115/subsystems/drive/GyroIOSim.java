package frc.team5115.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;
    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override // specified by GroIOSim interface
    public void updateInputs(GyroIOInputs inputs) {
        GyroIO.super.updateInputs(inputs);
    }
    public Rotation2d getGyroRotation() {
        return this.gyroSimulation.getGyroReading();
    }

    @Override // specified by GroIOSim interface
    
    public AngularVelocity getGyroAngularVelocity() {
        return this.gyroSimulation.getMeasuredAngularVelocity();
    }
}