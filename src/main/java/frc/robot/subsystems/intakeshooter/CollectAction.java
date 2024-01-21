package frc.robot.subsystems.intakeshooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCPositionAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

//
// Phase 1: Collect requested
//   - Move the collector down to collect position
//   - Start spinning the collector
//   - Move the shooter tilt to collect position
//
// Wait for note sensor or cancel
//
// Phase 2: Note detected
//   - Move the collector up to stowed position
//   - Stop spinning the collector
//
// Phase 3: Canceld
//   - Move the collector to stowed positoin
//   - Stop spinning the collector
//
public class CollectAction extends Action {
    private IntakeShooterSubsystem sub_ ;

    private MCPositionAction intake_down_ ;
    private MCVelocityAction spinner_on_ ;
    private MCPositionAction tilt_collect_ ;

    public CollectAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        intake_down_ = new MCPositionAction(sub_.updown(), "collect", "positions:collect" ) ;
        spinner_on_ = new MCVelocityAction(sub.spinner(), "collect", "velocities:collect") ;
        tilt_collect_ = new MCPositionAction(sub_.tilt(), "collect", "positions:collect") ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.updown().setAction(intake_down_, true) ;
        sub_.spinner().setAction(spinner_on_, true) ;
        sub_.tilt().setAction(tilt_collect_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    public String toString(int indent) {
        return spaces(indent) + "CollectAction" ;
    }
}
