package frc.robot.automodes;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.swerve.SwerveHolonomicDynamicPathAction;
import org.xero1425.base.subsystems.swerve.SwerveTrackAngle;
import org.xero1425.base.utils.Pose2dWithRotation;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.XeroMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.intake_shooter.IntakeAutoShootAction;
import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.intake_shooter.IntakeManualShootAction;
import frc.robot.subsystems.intake_shooter.StartCollectAltAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public abstract class AllegroAutoModeAction extends Action {

    private AllegroRobot2024 robot_ ;
    private IntakeGotoNamedPositionAction stow_ ;
    private IntakeManualShootAction manual_shoot_low_ ;
    private IntakeManualShootAction manual_shoot_ ;
    private IntakeAutoShootAction auto_shoot_ ;
    private StartCollectAltAction start_collect_ ;       
    private SwerveTrackAngle rotate_ ;
    private SwerveHolonomicDynamicPathAction current_path_ ;

    private boolean mirror_ ;
    private double mvalue_ ;
    
    public AllegroAutoModeAction(AllegroRobot2024 robot, boolean mirror, double mvalue) throws Exception {
        super(robot.getRobot()) ;

        mirror_ = mirror ;
        mvalue_ = mvalue ;

        robot_ = robot ;

        double v1 = robot_.getIntakeShooter().getUpDown().getSettingsValue("targets:stow").getDouble() ;
        double v2 = robot_.getIntakeShooter().getTilt().getSettingsValue("targets:stow").getDouble() ;         
        stow_ = new IntakeGotoNamedPositionAction(robot_.getIntakeShooter(), v1, v2) ;

        double updown = robot_.getIntakeShooter().getSettingsValue("actions:manual-shoot:subwoofer-center-low:updown").getDouble() ;
        double tilt = robot_.getIntakeShooter().getSettingsValue("actions:manual-shoot:subwoofer-center-low:tilt").getDouble() ;

        start_collect_ = new StartCollectAltAction(robot_.getIntakeShooter(), updown, tilt) ;
        manual_shoot_low_ = new IntakeManualShootAction(robot_.getIntakeShooter(), "subwoofer-center-low") ;          
        
        manual_shoot_ = new IntakeManualShootAction(robot_.getIntakeShooter(), "subwoofer-center");

        double rottol = robot.getIntakeShooter().getSettingsValue("actions:auto-shoot:rotational-position-tolerance").getDouble() ;
        rotate_ = new SwerveTrackAngle(robot.getSwerve(), () -> robot_.getTargetTracker().getRotation(), rottol) ;
        auto_shoot_ = new IntakeAutoShootAction(robot_.getIntakeShooter(), robot_.getTargetTracker(), true, rotate_) ;        
    }

    protected AllegroRobot2024 getRobotSubsystem() {
        return robot_ ;
    }

    protected XeroRobot getRobot() {
        return robot_.getRobot() ;
    }

    protected boolean hasNote() {
        return robot_.getIntakeShooter().isHoldingNote() ;
    }

    protected double adjustRotationRedBlue(double r) {
        if (mirror_) {
            r = XeroMath.normalizeAngleDegrees(180 - r) ;
        }

        return r ;
    }

    protected Pose2dWithRotation adjustPoseRedBlue(Pose2dWithRotation pose) {
        if (mirror_) {
            double h = XeroMath.normalizeAngleDegrees(180 - pose.getRotation().getDegrees()) ;
            double r = XeroMath.normalizeAngleDegrees(180 - pose.getRobotRotation().getDegrees()) ;
            double x = mvalue_ - pose.getTranslation().getX() ;
            double y = pose.getTranslation().getY() ;
            pose = new Pose2dWithRotation(x, y, Rotation2d.fromDegrees(h), Rotation2d.fromDegrees(r)) ;
        }

        return pose ;
    }

    protected IntakeGotoNamedPositionAction getStowAction() {
        return stow_ ;
    }

    protected IntakeManualShootAction getManualShootLowAction() {
        return manual_shoot_low_ ;
    }

    protected IntakeManualShootAction getManualShootAction() {
        return manual_shoot_ ;
    }

    protected IntakeAutoShootAction getAutoShootAction() {
        return auto_shoot_ ;
    }

    protected StartCollectAltAction getStartCollectAction() {
        return start_collect_ ;
    }

    protected SwerveTrackAngle getRotateAction() {
        return rotate_ ;
    }

    private Pose2dWithRotation getCurrentRobotPose() {
        Pose2d p = robot_.getSwerve().getPose() ;
        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Info, robot_.getSwerve().getLoggerID()) ;
        logger.add("getCurrentPose for path") ;
        logger.add("pose", p.toString()) ;
        logger.endMessage();
        return new Pose2dWithRotation(p.getX(), p.getY(), p.getRotation(), p.getRotation());
    }

    private SwerveHolonomicDynamicPathAction createDynamicPath(String name, double maxv, double maxa, double pre, double post, Pose2dWithRotation pts[]) throws Exception {
        Pose2dWithRotation allpts[] = new Pose2dWithRotation[pts.length + 1] ;
        allpts[0] = getCurrentRobotPose() ;
        for(int i = 0 ; i < pts.length ; i++) {
            allpts[i + 1] = pts[i] ;
        }

        return new SwerveHolonomicDynamicPathAction(robot_.getSwerve(), name, maxv, maxa, 0.2, pre, post, allpts) ;
    }

    protected boolean gotoPoseWithRotation(String name, double maxv, double maxa, double pre, double post, Pose2dWithRotation pts[]) {
        boolean ret = true ;
        try {
            current_path_ = createDynamicPath(name, maxv, maxa, pre, post, pts) ;
            robot_.getSwerve().setAction(current_path_, true) ;
        }
        catch(Exception ex) {
            current_path_ = null ;
            ret = false ;
        }

        return ret;
    }

    protected boolean gotoPoseWithRotation(String name, double maxv, double maxa, double pre, double post, Pose2dWithRotation dest) {
        return gotoPoseWithRotation(name, maxv, maxa, pre, post, new Pose2dWithRotation[] { dest });
    }

    protected boolean gotoPoseWithRotationAndCollect(String name, double maxv, double maxa, double pre, double post, Pose2dWithRotation pts[]) {
        if (!gotoPoseWithRotation(name, maxv, maxa, pre, post, pts))
            return false ;

        robot_.getIntakeShooter().setAction(start_collect_, true) ;

        return true ;
    }

    protected boolean gotoPoseWithRotationAndCollect(String name, double maxv, double maxa, double pre, double post, Pose2dWithRotation dest) {
        return gotoPoseWithRotationAndCollect(name, maxv, maxa, pre, post, new Pose2dWithRotation[] { dest });
    }    

    protected boolean gotoPoseWithRotationAndShoot(String name, double maxv, double maxa, double pre, double post, Pose2dWithRotation pts[], Translation2d loc, double dist) {
        if (!gotoPoseWithRotation(name, maxv, maxa, pre, post, pts))
            return false ;

        current_path_.addLocationBasedAction(loc, dist, ()->robot_.getIntakeShooter().setAction(manual_shoot_low_, true)) ;
        return true ;
    }

    protected boolean gotoPoseWithRotationAndShoot(String name, double maxv, double maxa, double pre, double post, Pose2dWithRotation dest, Translation2d loc, double dist) {
        return gotoPoseWithRotationAndShoot(name, maxv, maxa, pre, post, new Pose2dWithRotation[] { dest }, loc, dist);
    }      

    protected boolean isCurrentPathDone() {
        return current_path_.isDone() ;
    }
}
