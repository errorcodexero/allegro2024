package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;


public class IntakeGotoNamedPositionAction extends Action {

    private enum State {
        Idle,
        MovingTilt,
        MovingBoth,
        Done,
    }

    private IntakeShooterSubsystem sub_;
    private MCMotionMagicAction updown_action_ ;
    private MCMotionMagicAction tilt_only_action_ ;
    private MCMotionMagicAction tilt_action_ ;
    private double updown_target_ ;
    private double tilt_target_ ;
    private State state_ ;

    public IntakeGotoNamedPositionAction(IntakeShooterSubsystem sub, double updown, double tilt) throws Exception{
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;

        updown_action_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position" , updown, 3.0 , 1);
        tilt_action_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position" , tilt, 3.0 , 1);

        state_ = State.Idle ;

        updown_target_ = updown;
        tilt_target_ = tilt;
    }

    public IntakeGotoNamedPositionAction(IntakeShooterSubsystem sub, String updown, String tilt) throws Exception{
        this(sub, sub.getSettingsValue(updown).getDouble(), sub.getSettingsValue(tilt).getDouble());
    }    

    @Override
    public void start() throws Exception {
        super.start();

        state_ = State.Idle ;
        double updownpos = sub_.getUpDown().getPosition() ;
        double updownstow = sub_.getUpDown().getSettingsValue("targets:stow").getDouble() ;
        double tiltstow = sub_.getTilt().getSettingsValue("targets:stow").getDouble() ;

        if (updownpos < updownstow && updown_target_ > updownpos) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info).add("pos < stow").add("pos", updownpos).add("stow", updownstow).endMessage() ;

            //
            // The updown is not in the stowed position, we must move the tilt to a compatible
            // position, before we start the synchronous movement.
            //
            double ttarget = tiltstow + updownstow - updownpos ;
            tilt_only_action_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", ttarget, 3.0, 1.0) ;
            sub_.getTilt().setAction(tilt_only_action_, true) ;
            state_ = State.MovingTilt ;
        }
        else 
        {
            sub_.getUpDown().setAction(updown_action_, true);
            sub_.getTilt().setAction(tilt_action_, true);
            state_ = State.MovingBoth ;
        }
    }

    @Override
    public void run() throws Exception{
        super.run() ;

        switch(state_) {
            case Idle:
                break ;

            case MovingTilt:
                if (tilt_only_action_.isDone()) {
                    sub_.getUpDown().setAction(updown_action_, true);
                    sub_.getTilt().setAction(tilt_action_, true);
                    state_ = State.MovingBoth ;                    
                }
                break ;

            case MovingBoth:
                if (tilt_action_.isDone() && updown_action_.isDone()) {
                    setDone() ;
                }
                break ;

            case Done:
                break ;
        }
    }

    @Override
    public String toString(int indent){
        return spaces(indent) + "IntakeGotoNamedPositionAction updown " + updown_target_ + ", tilt " + tilt_target_ ;
    }   
}