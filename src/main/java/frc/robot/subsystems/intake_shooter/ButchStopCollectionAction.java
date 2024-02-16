package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class ButchStopCollectionAction extends Action {
    private IntakeShooterSubsystem sub_ ;

    private double updown_threshold_ ;
    private MCMotionMagicAction intake_up_ ;
    private MotorEncoderPowerAction spinner_feeder_off_ ;
    private MCMotionMagicAction tilt_stow_ ;
    private boolean tilt_running_ ;

    public ButchStopCollectionAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        updown_threshold_ = sub.getSettingsValue("actions:butch-stop-collect:updown-threshold").getDouble() ;

        intake_up_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position", "targets:stow", 1, 1) ;
        spinner_feeder_off_ = new MotorEncoderPowerAction(sub_.getFeeder(), 0.0) ;
        tilt_stow_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", "targets:stow", 1, 1);
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getFeeder().setAction(spinner_feeder_off_, true) ;
        sub_.getUpDown().setAction(intake_up_, true) ;
        tilt_running_ = false ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (sub_.getUpDown().getPosition() > updown_threshold_ && !tilt_running_) {
            tilt_running_ = true ;
            sub_.getTilt().setAction(tilt_stow_, true) ;
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
        return spaces(indent) + "ButchStopCollectAction" ;
    }       
}
