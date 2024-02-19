package frc.robot.subsystems.superstructure;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class ClimbAction extends Action {
    public enum ClimbType {
        HooksUp,
        HooksDown,
        HooksDownWithRobot
    } ;

    private MotorEncoderSubsystem climb_ ;
    private ClimbType type_ ;
    private double target_ ;
    private double power_ ;

    public ClimbAction(MotorEncoderSubsystem climb, ClimbType type) throws Exception {
        super(climb.getRobot().getMessageLogger());
        climb_ = climb;
        type_ = type ;

        switch(type_) {
            case HooksUp:
                power_ = climb_.getSettingsValue("actions:climb:hooks-up:power").getDouble() ;
                target_ = climb_.getSettingsValue("actions:climb:hooks-up:target").getDouble() ;
                break ;
            case HooksDown:
                power_ = climb_.getSettingsValue("actions:climb:hooks-down:power").getDouble() ;
                target_ = climb_.getSettingsValue("actions:climb:hooks-down:target").getDouble() ;                
                break ;
            case HooksDownWithRobot:
                power_ = climb_.getSettingsValue("actions:climb:hooks-down-with-robot:power").getDouble() ;
                target_ = climb_.getSettingsValue("actions:climb:hooks-down-with-robot:target").getDouble() ;                
                break ;
        }
    }

    @Override
    public void start() throws Exception {
        super.start();

        if (isAtTargetPosition()) {
            setDone() ;
        }
        else {
            climb_.setPower(power_) ;
        }
    }

    private boolean isAtTargetPosition() {
        double pos = climb_.getPosition() ;

        if (type_ == ClimbType.HooksUp && pos >= target_) {
            return true ;
        }

        if (type_ == ClimbType.HooksDown && pos <= target_) {
            return true ;
        }

        if (type_ == ClimbType.HooksDownWithRobot && pos <= target_) {
            return true ;
        }

        return false ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (isAtTargetPosition()) {
            //
            // We are done, stop the climber
            //
            climb_.setPower(0.0) ;
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel();
        climb_.setPower(0.0) ;        
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ClimbAction " + type_.toString() + ", target = " + target_ ;
    }
}
