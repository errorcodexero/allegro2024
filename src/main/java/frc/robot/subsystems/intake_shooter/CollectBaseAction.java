package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.misc.MessageType;

public abstract class CollectBaseAction extends Action {
    private enum StowStates {
        BringIntakeIn,
        StowingUpDown,
        StowingBoth
    };

    private IntakeShooterSubsystem sub_;
    private StowStates state_;
    private double updown_up_threshold_;

    private MCMotionMagicAction tilt_bring_in_ ;
    private MCMotionMagicAction tilt_stow_;
    private MCMotionMagicAction updown_stow_;

    public CollectBaseAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;
        updown_up_threshold_ = sub.getSettingsValue("actions:butch-start-collect:updown-up-threshold").getDouble();
        tilt_stow_ = new MCMotionMagicAction(getSubsystem().getTilt(), "pids:position", "targets:stow", 1, 1);
        tilt_bring_in_ = new MCMotionMagicAction(getSubsystem().getTilt(), "pids:position", "targets:collect", 1, 1);
        updown_stow_ = new MCMotionMagicAction(getSubsystem().getUpDown(), "pids:position", "targets:stow", 1, 1);
    }

    protected IntakeShooterSubsystem getSubsystem() {
        return sub_;
    }

    protected void startStow() {
        state_ = StowStates.BringIntakeIn;
        getSubsystem().getTilt().setAction(tilt_bring_in_, true);

    }

    protected void runStow() {
        StowStates prev = state_;

        switch (state_) {
            case BringIntakeIn:
                if (tilt_bring_in_.isDone()) {
                    getSubsystem().getUpDown().setAction(updown_stow_, true);                    
                    state_ = StowStates.StowingUpDown;
                }
                break;
            case StowingUpDown:
                if (getSubsystem().getUpDown().getPosition() > updown_up_threshold_) {
                    getSubsystem().getTilt().setAction(tilt_stow_, true);
                    state_ = StowStates.StowingBoth;
                }
                break;

            case StowingBoth:
                if (tilt_stow_.isDone() && updown_stow_.isDone()) {
                    setDone();
                }
                break;
        }

        if (state_ != prev) {
            getSubsystem().getRobot().getMessageLogger().startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            getSubsystem().getRobot().getMessageLogger().add("stow state changed: ");
            getSubsystem().getRobot().getMessageLogger().add(prev.toString()) ;
            getSubsystem().getRobot().getMessageLogger().add(" ->");            
            getSubsystem().getRobot().getMessageLogger().add(state_.toString()) ;            
            getSubsystem().getRobot().getMessageLogger().endMessage();
        }        
    }
}
