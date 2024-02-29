package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class StopCollectAction extends CollectBaseAction {

    IntakeShooterSubsystem sub_ ;
    private MotorEncoderPowerAction feeder_off_ ;

    public StopCollectAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub) ;

        sub_ = sub ;
        feeder_off_ = new MotorEncoderPowerAction(sub.getFeeder(), 0.0) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        sub_.getFeeder().setAction(feeder_off_, true) ;
        startStow();
    }

    @Override
    public void run() throws Exception {
        super.run() ;
        runStow();
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    public String toString(int indent) {
        return spaces(indent) + "ButchStopCollectAction" ;
    }       
}
