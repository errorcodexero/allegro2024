package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class StopCollectAction extends Action {
    private IntakeShooterSubsystem sub_ ;

    private double tilt_threshold_ ;
    private MCMotionMagicAction intake_up_ ;
    private MotorEncoderPowerAction spinner_feeder_off_ ;
    private MCMotionMagicAction tilt_stow_ ;

    public StopCollectAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        tilt_threshold_ = sub.getSettingsValue("actions:stow:tilt-threshold").getDouble() ;

        intake_up_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position", "targets:stow", 1, 1) ;
        spinner_feeder_off_ = new MotorEncoderPowerAction(sub_.getFeeder(), 0.0) ;
        tilt_stow_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", "targets:stow", 1, 1);
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getFeeder().setAction(spinner_feeder_off_, true) ;
        sub_.getTilt().setAction(tilt_stow_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (sub_.getTilt().getPosition() < tilt_threshold_) {
            sub_.getUpDown().setAction(intake_up_, true) ;
        }

        if (intake_up_.isDone() && tilt_stow_.isDone()) {
            setDone();
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    public String toString(int indent) {
        return spaces(indent) + "CollectAction" ;
    }    
}
