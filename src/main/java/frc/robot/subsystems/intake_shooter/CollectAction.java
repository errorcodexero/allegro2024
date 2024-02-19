package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.misc.MessageType;

public abstract class CollectAction extends Action {
    private enum StowStates {
        StowingUpDown,
        StowingBoth
    };

    private IntakeShooterSubsystem sub_;
    private StowStates state_;
    private double updown_up_threshold_;

    private MCMotionMagicAction tilt_stow_;
    private MCMotionMagicAction updown_stow_;

    public CollectAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;
        updown_up_threshold_ = sub.getSettingsValue("actions:butch-start-collect:updown-up-threshold").getDouble();
        tilt_stow_ = new MCMotionMagicAction(getSubsystem().getTilt(), "pids:position", "targets:stow", 1, 1);
        updown_stow_ = new MCMotionMagicAction(getSubsystem().getUpDown(), "pids:position", "targets:stow", 1, 1);
    }

    protected IntakeShooterSubsystem getSubsystem() {
        return sub_;
    }

    protected void startStow() {
        state_ = StowStates.StowingUpDown;
        getSubsystem().getUpDown().setAction(updown_stow_);
    }

    protected void runStow() {
        StowStates prev = state_;

        switch (state_) {
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
