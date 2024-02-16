package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;

public class AmpTrapPositionAction extends Action {
    private enum State {
        Idle,
        CrossMinToMax,
        CrossMaxToMin,
        CrossMinToMaxWaitClear,
        CrossMaxToMinWaitClear,
        DirectToTarget
    }

    private AmpTrapSubsystem sub_ ;
    private double keep_out_min_ ;
    private double keep_out_max_ ;
    private double keep_out_height_ ;
    private double target_angle_ ;
    private double target_height_ ;

    private MCMotionMagicAction keepout_height_action_ ;
    private MCMotionMagicAction arm_goto_min_action_ ;
    private MCMotionMagicAction arm_goto_max_action_ ;
    private MCMotionMagicAction arm_goto_target_action_ ;
    private MCMotionMagicAction elevator_goto_target_action_ ;

    private State state_ ;

    public AmpTrapPositionAction(AmpTrapSubsystem sub, double angle, double height) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        keep_out_height_ = sub_.getSettingsValue("props:keepout-zone:elevator-height").getDouble() ;
        keep_out_min_ = sub_.getSettingsValue("props:keepout-zone:min").getDouble() ;
        keep_out_max_ = sub_.getSettingsValue("props:keepout-zone:max").getDouble() ;  
        
        target_height_ = height ;
        target_angle_ = angle ;

        keepout_height_action_ = new MCMotionMagicAction(sub_.getElevator(), "pids:position", keep_out_height_, 0.05, 0.05) ;
        arm_goto_min_action_ = new MCMotionMagicAction(sub_.getArm(), "pids:position", keep_out_min_, 5.0, 1.0) ;
        arm_goto_max_action_ = new MCMotionMagicAction(sub_.getArm(), "pids:position", keep_out_max_, 5.0, 1.0) ;

        arm_goto_target_action_ = new MCMotionMagicAction(sub_.getArm(), "pids:position", target_angle_, 5.0, 1.0) ;
        elevator_goto_target_action_ = new MCMotionMagicAction(sub_.getElevator(), "pids:position", target_height_, 0.1, 0.05) ;

        state_ = State.Idle ;
    }

    public AmpTrapPositionAction(AmpTrapSubsystem sub, String pivot, String height) throws Exception {
        this(sub, sub.getSettingsValue(pivot).getDouble(), sub.getSettingsValue(height).getDouble()) ;
    }    

    @Override
    public void start() throws Exception {
        super.start() ;
        double current = sub_.getArm().getPosition() ;
        if (current < keep_out_min_ && target_angle_ > keep_out_max_) {
            //
            // We need to cross the keep out zone from min to max
            //
            if (target_height_ > keep_out_height_) {
                sub_.getElevator().setAction(elevator_goto_target_action_, true) ;
            } else {
                sub_.getElevator().setAction(keepout_height_action_, true) ;
            }
            sub_.getArm().setAction(arm_goto_min_action_, true) ;
            state_ = State.CrossMinToMax ;
        }
        else if (current > keep_out_max_ && target_angle_ < keep_out_min_) {
            //
            // We need to cross the keep out zone from max to min
            //
            if (target_height_ > keep_out_height_) {
                sub_.getElevator().setAction(elevator_goto_target_action_, true) ;
            } else {
                sub_.getElevator().setAction(keepout_height_action_, true) ;
            }
            sub_.getArm().setAction(arm_goto_max_action_, true) ;
            state_ = State.CrossMaxToMin ;
        }
        else {
            //
            // We can go directly to the targets
            //
            sub_.getElevator().setAction(elevator_goto_target_action_, true) ;
            sub_.getArm().setAction(arm_goto_target_action_, true) ;
            state_ = State.DirectToTarget ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;
        if (state_ == State.CrossMinToMax) {
            if (sub_.getElevator().getPosition() >= keep_out_height_) {
                sub_.getArm().setAction(arm_goto_target_action_, true) ;
            }

            if (target_height_ > keep_out_height_) {
                state_ = State.DirectToTarget ;
            }
            else {
                state_ = State.CrossMinToMaxWaitClear ;
            }
        } else if (state_ == State.CrossMaxToMin) {
            if (sub_.getElevator().getPosition() >= keep_out_height_) {
                sub_.getArm().setAction(arm_goto_target_action_, true) ;
            }

            if (target_height_ > keep_out_height_) {
                state_ = State.DirectToTarget ;
            }
            else {
                state_ = State.CrossMaxToMinWaitClear ;
            }            
        }
        else if (state_ == State.CrossMinToMaxWaitClear) {
            if (sub_.getArm().getPosition() >= keep_out_max_) {
                sub_.getElevator().setAction(elevator_goto_target_action_, true) ;
                state_ = State.DirectToTarget ;
            }
        }
        else if (state_ == State.CrossMaxToMinWaitClear) {
            if (sub_.getArm().getPosition() <= keep_out_min_) {
                sub_.getElevator().setAction(elevator_goto_target_action_, true) ;
                state_ = State.DirectToTarget ;
            }
        }
        else if (state_ == State.DirectToTarget) {
            if (elevator_goto_target_action_.isDone() && arm_goto_target_action_.isDone()) {
                setDone() ;
            }
        }
    }
    
    @Override
    public String toString(int indent) {
        return spaces(indent) + "AmpTrapPositionAction, angle=" + target_angle_ + ", height " + target_height_ ;
    }
}
