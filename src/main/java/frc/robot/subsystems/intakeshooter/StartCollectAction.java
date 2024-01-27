package frc.robot.subsystems.intakeshooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

//
// Phase 1: Collect requested
//   - Move the shooter/intake down to the ground (updown motor)
//   - Move the shooter tilt to collect position (tilt motor)
//   - Start spinning the intake rollers (intake motor)
//   - Start spinning the keeper rollers (keeper motor)
//
// Wait for note sensor or cancel
//
// Phase 2: Note detected
//   - Move the collector up to stowed position
//   - Stop spinning the collector
//
// Phase 2: Canceled
//   - Move the collector to stowed positoin
//   - Stop spinning the collector
//
public class StartCollectAction extends Action {
    private IntakeShooterSubsystem sub_ ;

    private double tilt_threshold_ ;
    private MCMotionMagicAction intake_down_ ;
    private MCVelocityAction spinner_feeder_on_ ;
    private MCMotionMagicAction tilt_collect_ ;

    public StartCollectAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        tilt_threshold_ = sub.getSettingsValue("actions:collect:tilt-threshold").getDouble() ;

        intake_down_ = new MCMotionMagicAction(sub_.updown(), "pids:position", "targets:collect", 1, 1) ;
        spinner_feeder_on_ = new MCVelocityAction(sub_.spinner_feeder(), "pids:velocity", "targets:collect") ;
        tilt_collect_ = new MCMotionMagicAction(sub_.tilt(), "pids:position", "targets:collect", 1, 1) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.spinner_feeder().setAction(spinner_feeder_on_, true) ;
        sub_.tilt().setAction(tilt_collect_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (sub_.tilt().getPosition() > tilt_threshold_) {
            sub_.updown().setAction(intake_down_, true) ;
        }

        if (sub_.isNotePresent()) {
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    public String toString(int indent) {
        return spaces(indent) + "StartCollectAction" ;
    }
}
