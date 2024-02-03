package frc.robot.subsystems.intake_shooter;

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
// Wait for note detected and then mark the action as done.
// This action is paired with the stop collect action.
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

        tilt_threshold_ = sub.getSettingsValue("actions:start-collect:tilt-threshold").getDouble() ;

        intake_down_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position", "targets:collect", 1, 1) ;
        spinner_feeder_on_ = new MCVelocityAction(sub_.getFeeder(), "pids:velocity", "targets:collect") ;
        tilt_collect_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", "targets:collect", 1, 1) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getFeeder().setAction(spinner_feeder_on_, true) ;
        sub_.getTilt().setAction(tilt_collect_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (sub_.getTilt().getPosition() > tilt_threshold_) {
            sub_.getUpDown().setAction(intake_down_, true) ;
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
