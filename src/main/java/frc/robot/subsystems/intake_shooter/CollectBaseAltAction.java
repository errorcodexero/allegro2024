package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;

public abstract class CollectBaseAltAction extends IntakeGotoBaseAction {

    public CollectBaseAltAction(IntakeShooterSubsystem sub, double updown, double tilt) throws Exception {
        super(sub);
    }

    protected void startStow() throws Exception {
        if (getSubsystem().isHoldingNote()) {
            gotoPosition("targets:shoot", "targets:shoot") ;
        }
        else {
            gotoPosition("targets:stow", "targets:stow") ;
        }
    }

    protected void runStow() throws BadMotorRequestException, MotorRequestFailedException {
        runGoto() ;
        if (isAtTarget()) {
            setDone() ;
        }
    }
}
