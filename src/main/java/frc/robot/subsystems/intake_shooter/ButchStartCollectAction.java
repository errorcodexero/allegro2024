package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

public class ButchStartCollectAction extends Action {
    private IntakeShooterSubsystem sub_ ;

    private boolean up_down_started_ ;
    private double tilt_threshold_ ;
    private MCMotionMagicAction intake_down_ ;
    private MCVelocityAction spinner_feeder_on_ ;
    private MCMotionMagicAction tilt_collect_ ;

    public ButchStartCollectAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        tilt_threshold_ = sub.getSettingsValue("actions:butch-start-collect:tilt-threshold").getDouble() ;

        intake_down_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position", "targets:collect", 1, 1) ;
        spinner_feeder_on_ = new MCVelocityAction(sub_.getFeeder(), "pids:velocity", "targets:collect", false) ;
        tilt_collect_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", "targets:collect", 1, 1) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        up_down_started_ = false ;
        sub_.getFeeder().setAction(spinner_feeder_on_, true) ;
        sub_.getTilt().setAction(tilt_collect_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (sub_.getTilt().getPosition() > tilt_threshold_ && up_down_started_ == false) {
            sub_.getUpDown().setAction(intake_down_, true) ;
            up_down_started_ = true ;
        }

        if (sub_.isNotePresent()) {
            sub_.getFeeder().setPower(0.0) ;
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    public String toString(int indent) {
        return spaces(indent) + "ButchStartCollectAction" ;
    }    
}
