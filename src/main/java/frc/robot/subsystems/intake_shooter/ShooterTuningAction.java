package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class ShooterTuningAction extends Action {
    private final double kTiltPositionTolerance = 1.0 ;
    private final double kTiltVelocityTolerance = 1.0 ;

    private IntakeShooterSubsystem sub_ ;
    private SimpleWidget tilt_widget_ ;
    private SimpleWidget velocity_widget_ ;
    private SimpleWidget toggle_widget_ ;
    private boolean last_toggle_value_ ;

    private MCVelocityAction velocity1_action_ ;
    private MCVelocityAction velocity2_action_ ;
    private MCMotionMagicAction tilt_action_ ;
    private ShuffleboardTab tab_ ;

    private double current_tilt_ ;
    private double current_velocity_ ;

    public ShooterTuningAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        velocity1_action_ = new MCVelocityAction(sub_.getShooter1(), "pids:velocity", 0.0);
        velocity2_action_ = new MCVelocityAction(sub_.getShooter2(), "pids:velocity", 0.0);
        tilt_action_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", 0.0, kTiltPositionTolerance, kTiltVelocityTolerance) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        last_toggle_value_ = false ;
        tab_ = Shuffleboard.getTab("Tuning");
        tilt_widget_ = tab_.add("Tilt Input", 0.0).withWidget(BuiltInWidgets.kTextView) ;
        velocity_widget_ = tab_.add("Velocity Input", 0.0).withWidget(BuiltInWidgets.kTextView) ;
        toggle_widget_ = tab_.add("Apply", false).withWidget(BuiltInWidgets.kToggleButton) ;

        tab_.addDouble("VSET", () -> { return current_velocity_ ;});
        tab_.addDouble("TSET", () -> { return current_tilt_ ;});
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        boolean tw = toggle_widget_.getEntry().getBoolean(false);
        if (tw == false && last_toggle_value_ == true) {
            current_velocity_= velocity_widget_.getEntry().getDouble(0.0) ;
            velocity1_action_.setTarget(current_velocity_);
            velocity2_action_.setTarget(current_velocity_);

            current_tilt_ = tilt_widget_.getEntry().getDouble(0.0) ;
            tilt_action_.setTarget(current_tilt_);
        }
        
        last_toggle_value_ = tw ;
    }
    

    public String toString(int indent) {
        return prefix(indent) + "TuningAction" ;
    }
}
