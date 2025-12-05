package frc.team5115.subsystems.catcher;

import edu.wpi.first.wpilibj.PWM;
import frc.team5115.Constants;

public class CatcherIOReal implements CatcherIO {
    private PWM linearActuator1;
    private PWM linearActuator2;

    private final double RETRACTED_POS = 0.0;
    private final double EXTENDED_POS = 1.0; // TODO adjust when we get the real robot

    private boolean state;

    public CatcherIOReal() {
        linearActuator1 = new PWM(Constants.NET_ACTUATOR_1_ID);
        linearActuator2 = new PWM(Constants.NET_ACTUATOR_2_ID);
    }

    @Override
    public void updateInputs(CatcherIOInputs inputs) {
        inputs.state = state;
    }

    @Override
    public void extendNet() {
        linearActuator1.setPosition(EXTENDED_POS);
        linearActuator2.setPosition(EXTENDED_POS);
        state = true;
    }

    @Override
    public void retractNet() {
        linearActuator1.setPosition(RETRACTED_POS);
        linearActuator2.setPosition(RETRACTED_POS);
        state = false;
    }
}
