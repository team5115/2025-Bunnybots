package frc.team5115.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        GyroIO.super.updateInputs(inputs);
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.angularVelocity = gyroSimulation.getMeasuredAngularVelocity();
    }
}
