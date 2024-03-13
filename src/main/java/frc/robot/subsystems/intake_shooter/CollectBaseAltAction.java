package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;

public abstract class CollectBaseAltAction extends Action {

    private IntakeShooterSubsystem sub_;

    private IntakeGotoNamedPositionAction stow_ ;
    private IntakeGotoNamedPositionAction shoot_ ;
    private IntakeGotoNamedPositionAction act_ ;

    public CollectBaseAltAction(IntakeShooterSubsystem sub, double updown, double tilt) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;

        if (Double.isNaN(updown) || Double.isNaN(tilt)) {
            tilt = sub_.getTilt().getSettingsValue("targets:stow-note").getDouble() ;
            updown = sub_.getUpDown().getSettingsValue("targets:stow").getDouble() ;
        }
        shoot_ = new IntakeGotoNamedPositionAction(sub_, updown, tilt) ;

        tilt = sub_.getTilt().getSettingsValue("targets:stow").getDouble() ;
        updown = sub_.getUpDown().getSettingsValue("targets:stow").getDouble() ;
        stow_ = new IntakeGotoNamedPositionAction(sub_, updown, tilt) ;
    }

    protected IntakeShooterSubsystem getSubsystem() {
        return sub_;
    }

    protected void startStow() {
        if (sub_.isHoldingNote())
            act_ = shoot_ ;
        else
            act_ = stow_ ;

        getSubsystem().setAction(act_, true);
    }

    protected void runStow() {
        if (act_.isDone()) {
            setDone() ;
        }
    }
}
