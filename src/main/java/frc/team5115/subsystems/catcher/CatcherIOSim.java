package frc.team5115.subsystems.catcher;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;

public class CatcherIOSim implements CatcherIO {

    private final DoubleSolenoidSim extenderSim;
    private boolean state = false;

    public CatcherIOSim() {
        extenderSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, 0, 0);
    }

    @Override
    public void updateInputs(CatcherIOInputs inputs) {
        inputs.state = state;
    }

    @Override
    public void extendNet() {
        state = true;
    }

    @Override
    public void retractNet() {
        state = false;
    }
}
