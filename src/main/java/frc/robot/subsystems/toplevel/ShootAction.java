package frc.robot.subsystems.toplevel;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.swerve.SwerveRotateToAngle;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.XeroMath;

import frc.robot.subsystems.intake_shooter.IntakeShootAction;

public class ShootAction extends Action {

    private AllegroRobot2024 robot_;
    private SwerveRotateToAngle rotate_;
    private IntakeShootAction shoot_ ;

    public ShootAction(AllegroRobot2024 robot) throws Exception {
        super(robot.getRobot().getMessageLogger());
        robot_ = robot;
        shoot_ = new IntakeShootAction(robot_.getIntakeShooter(), robot_.getTargetTracker()) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        MessageLogger logger = robot_.getRobot().getMessageLogger() ;

        double angle = XeroMath.normalizeAngleDegrees(robot_.getTargetTracker().getRotation() + robot_.getSwerve().getPose().getRotation().getDegrees());
        rotate_ = new SwerveRotateToAngle(robot_.getSwerve(), angle) ;
        robot_.getSwerve().setAction(rotate_, true);

        robot_.getIntakeShooter().setAction(shoot_, true) ;

        logger.startMessage(MessageType.Info);
        logger.add("Shooting:");
        logger.add("dbangle", angle);
        logger.endMessage();
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (rotate_.isDone()) {
            shoot_.setDBReady(true) ;
        }

        if (shoot_.isDone()) {
            setDone();
        }
    }

    @Override
    public void cancel() {
        super.cancel();
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "ShootAction";
    }
}
