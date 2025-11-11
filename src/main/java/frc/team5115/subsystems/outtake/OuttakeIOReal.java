package frc.team5115.subsystems.outtake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.team5115.Constants;

public class OuttakeIOReal implements OuttakeIO {
    private final DoubleSolenoid extender;
    private boolean state;

    public OuttakeIOReal(PneumaticHub hub) {
        extender =
                hub.makeDoubleSolenoid(
                        Constants.OUTTAKE_FORWARD_CHANNEL, Constants.OUTTAKE_REVERSE_CHANNEL);
    }

    @Override
    public void updateInputs(OuttakeIOInputs inputs) {
        inputs.state = state;
    }

    @Override
    public void setPneumatic(boolean extend) {
        state = extend;
        extender.set(extend ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }
}
