package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class AmpTrapPositionAction extends Action {
    private enum State {
        Idle,                           // Doing nothing
        CrossMinToMax,                  // The arm needs to cross the keepout zone from min to max
        CrossMaxToMin,                  // The arm needs to cross the keepout zone from max to min
        CrossMinToMaxWaitClear,         // The arm is actively crossing from min to max, waiting for the arm to clear the keepout zone
        CrossMaxToMinWaitClear,         // The arm is actively crossing from max to min, waiting for the arm to clear the keepout zone
        DirectToTarget                  // Everything is clear, just waiting for the actions to complete
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

    public 
    AmpTrapPositionAction(AmpTrapSubsystem sub, double angle, double height) throws Exception {
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
                //
                // The eventual height is above the keep out zone, so we can go directly to the target height
                // and just monitor the elevator height and start the arm when we get above the keepout height
                //
                sub_.getElevator().setAction(elevator_goto_target_action_, true) ;
            } else {
                //
                // The eventual height is below the keepout zone, so we need to go to the keepout height,
                // move the arm, and then go to the target height
                //
                sub_.getElevator().setAction(keepout_height_action_, true) ;
            }

            //
            // In either case we are going to move the arm to the min position and wait for the elevator
            // to be above the keepout height before moving the arm to the target position
            //
            sub_.getArm().setAction(arm_goto_min_action_, true) ;
            state_ = State.CrossMinToMax ;
        }
        else if (current > keep_out_max_ && target_angle_ < keep_out_min_) {
            //
            // We need to cross the keep out zone from max to min
            //
            if (target_height_ > keep_out_height_) {
                //
                // The eventual height is above the keep out zone, so we can go directly to the target height
                // and just monitor the elevator height and start the arm when we get above the keepout height
                //
                sub_.getElevator().setAction(elevator_goto_target_action_, true) ;
            } else {
                //
                // The eventual height is below the keepout zone, so we need to go to the keepout height,
                // move the arm, and then go to the target height
                //
                sub_.getElevator().setAction(keepout_height_action_, true) ;
            }

            //
            // In either case we are going to move the arm to the max position and wait for the elevator
            // to be above the keepout height before moving the arm to the target position
            //            
            sub_.getArm().setAction(arm_goto_max_action_, true) ;
            state_ = State.CrossMaxToMin ;
        }
        else {
            //
            // There is no interference with the keepout zone, both elevator and arm can go directly to the target
            //
            sub_.getElevator().setAction(elevator_goto_target_action_, true) ;
            sub_.getArm().setAction(arm_goto_target_action_, true) ;
            state_ = State.DirectToTarget ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        State prev = state_ ;

        if (state_ == State.CrossMinToMax) {
            //
            // We are going from the min to max position, we need to wait for the elevator to be above the keepout
            // height before we can move the arm to the target position
            //
            if (sub_.getElevator().getPosition() >= keep_out_height_ - 0.1) {
                sub_.getArm().setAction(arm_goto_target_action_, true) ;

                if (target_height_ > keep_out_height_) {
                    state_ = State.DirectToTarget ;
                }
                else {
                    state_ = State.CrossMinToMaxWaitClear ;
                }                
            }
        } else if (state_ == State.CrossMaxToMin) {
            //
            // We are going from the max to min position, we need to wait for the elevator to be above the keepout
            // height before we can move the arm to the target position
            //
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
            //
            // Wait for the ARM to clear the keepout zone and then have the elevator go to the
            // the target height
            //
            if (sub_.getArm().getPosition() >= keep_out_max_) {
                sub_.getElevator().setAction(elevator_goto_target_action_, true) ;
                state_ = State.DirectToTarget ;
            }
        }
        else if (state_ == State.CrossMaxToMinWaitClear) {
            //
            // Wait for the ARM to clear the keepout zone and then have the elevator go to the
            // the target height
            //
            if (sub_.getArm().getPosition() <= keep_out_min_) {
                sub_.getElevator().setAction(elevator_goto_target_action_, true) ;
                state_ = State.DirectToTarget ;
            }
        }
        else if (state_ == State.DirectToTarget) {
            //
            // Everything is all clear, wait for the actions to complete
            //
            if (elevator_goto_target_action_.isDone() && arm_goto_target_action_.isDone()) {
                setDone() ;
            }
        }

        if (prev != state_) {
            MessageLogger logger = sub_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug) ;
            logger.add("AmpTrapPositionState changed : " + prev.toString() + " -> " + state_.toString()) ;
            logger.endMessage();            
        }
    }
    
    @Override
    public String toString(int indent) {
        return spaces(indent) + "AmpTrapPositionAction, angle=" + target_angle_ + ", height " + target_height_ ;
    }
}
