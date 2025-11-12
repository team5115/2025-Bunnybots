package frc.team5115.subsystems.catcher;

public class CatcherIOSim implements CatcherIO {

    private boolean state = false;

    public CatcherIOSim() {}

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
