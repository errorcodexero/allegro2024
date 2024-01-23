package frc.robot.subsystems.intakeshooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCPositionAction;
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
public class CollectAction extends Action {
    private IntakeShooterSubsystem sub_ ;

    private MCPositionAction intake_down_ ;
    private MCVelocityAction spinner_on_ ;
    private MCVelocityAction feeder_on_ ;
    private MCPositionAction tilt_collect_ ;

    public CollectAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        intake_down_ = new MCPositionAction(sub_.updown(), "pids:position", "targets:collect" ) ;
        spinner_on_ = new MCVelocityAction(sub_.spinner(), "pids:velocity", "targets:collect") ;
        tilt_collect_ = new MCPositionAction(sub_.tilt(), "pids:position", "targets:collect") ;
        feeder_on_ = new MCVelocityAction(sub_.feeder(), "pids:velocity", "targets:collect") ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.updown().setAction(intake_down_, true) ;
        sub_.spinner().setAction(spinner_on_, true) ;
        sub_.tilt().setAction(tilt_collect_, true) ;
        sub_.feeder().setAction(feeder_on_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    @Override
    public void cancel() {
        
    }

    public String toString(int indent) {
        return spaces(indent) + "CollectAction" ;
    }
}
