package frc.team5115.subsystems.outtake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;

public class OuttakeIOSim implements OuttakeIO {
    private final DoubleSolenoidSim extenderSim;
    private boolean state = false;

    public OuttakeIOSim() {
        extenderSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, 0, 0);
    }

    @Override
    public void updateInputs(OuttakeIOInputs inputs) {
        inputs.state = state;
    }

    @Override
    public void setPneumatic(boolean extend) {
        state = extend;
        extenderSim.set((extend ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse));
    }
}
