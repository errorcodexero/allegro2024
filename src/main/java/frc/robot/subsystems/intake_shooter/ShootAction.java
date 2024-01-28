package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.utils.PieceWiseLinear;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;

public class ShootAction extends Action {
    private IntakeShooterSubsystem sub_ ;
    private TargetTrackerSubsystem tt_ ;
    private PieceWiseLinear tilt_pwl_ ;
    private PieceWiseLinear shooter1_pwl_ ;
    private PieceWiseLinear shooter2_pwl_ ;    

    private double target_s1_ ;
    private double target_s2_ ;
    private double target_tilt_ ;
    private double distance_ ;

    private double shoot_margin_ ;
    private double tilt_margin_ ;
    private double angle_margin_ ;

    private MCVelocityAction shoot1_velocity_ ;
    private MCVelocityAction shoot2_velocity_ ;
    private MCMotionMagicAction tilt_position_ ;
    private MCVelocityAction feed_velocity_ ;

    private XeroTimer shoot_timer_ ;
    private boolean shooting_ ;
    
    public ShootAction(IntakeShooterSubsystem sub, TargetTrackerSubsystem tt) throws Exception {
        super(sub.getRobot().getMessageLogger());
        String name ;

        sub_ = sub ;
        shooting_ = false ;
        tt_ = tt ;

        shoot_margin_ = sub.getSettingsValue("actions:shoot:shoot-margin").getDouble() ;
        tilt_margin_ = sub.getSettingsValue("actions:shoot:tilt-margin").getDouble() ;
        angle_margin_ = sub.getSettingsValue("actions:shoot:angle-margin").getDouble() ;

        name = "subsystems:tilt:parameters:tilt-pwl" ;
        tilt_pwl_ = new PieceWiseLinear(sub.getRobot().getSettingsSupplier(), name) ;

        name = "subsystems:shooter1:parameters:velocity-pwl" ;
        shooter1_pwl_ = new PieceWiseLinear(sub.getRobot().getSettingsSupplier(), name) ;
        
        name = "subsystems:shooter2:parameters:velocity-pwl" ;
        shooter2_pwl_ = new PieceWiseLinear(sub.getRobot().getSettingsSupplier(), name) ;        

        shoot1_velocity_ = new MCVelocityAction(sub_.getShooter1(), "pids:velocity", 0.0) ;
        shoot2_velocity_ = new MCVelocityAction(sub_.getShooter1(), "pids:velocity", 0.0) ;
        tilt_position_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", 1.0, 1, 1) ;
        feed_velocity_ = new MCVelocityAction(sub_.getFeeder(), "pids:velocity", "targets:shoot");

        shoot_timer_ = new XeroTimer(sub.getRobot(), "shoot", 1.0) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        if (!tt_.hasSpeakerLocation()) {
            MessageLogger logger = sub_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("could not start ShootAction as target tracker does not have speaker location") ;
            logger.endMessage();
            setDone() ;
        }
        else {
            sub_.getShooter1().setAction(shoot1_velocity_, true) ;
            sub_.getShooter2().setAction(shoot2_velocity_, true) ;
            sub_.getTilt().setAction(tilt_position_, true) ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (!shooting_) {
            distance_ = tt_.getAngleToSpeaker() ;

            target_s1_ = shooter1_pwl_.getValue(distance_) ;
            target_s2_ = shooter2_pwl_.getValue(distance_) ;
            target_tilt_ = tilt_pwl_.getValue(distance_) ;

            shoot1_velocity_.setTarget(target_s1_);
            shoot2_velocity_.setTarget(target_s2_);
            tilt_position_.setTarget(target_tilt_);
            
            if (tiltIsReady() && shooterIsReady() && dbIsReady()) {
                sub_.getFeeder().setAction(feed_velocity_, true) ;
                shoot_timer_.start() ;
                shooting_ = true ;
            }
        }
        else {
            if (shoot_timer_.isExpired()) {
                sub_.getShooter1().setPower(0.0);
                sub_.getShooter2().setPower(0.0);
                sub_.getFeeder().setPower(0.0);
                setDone() ;
            }
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ShootAction" ;
    }

    private boolean tiltIsReady() {
        return Math.abs(sub_.getTilt().getPosition() - target_tilt_) < tilt_margin_ ;
    }

    public boolean shooterIsReady() {
        return Math.abs(sub_.getShooter1().getVelocity() - target_s1_) < shoot_margin_ &&
               Math.abs(sub_.getShooter2().getVelocity() - target_s2_) < shoot_margin_ ;
    }

    public boolean dbIsReady() {
        return Math.abs(tt_.getAngleToSpeaker()) < angle_margin_ ;
    }
}
