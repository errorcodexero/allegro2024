package frc.robot.automodes;

import org.xero1425.base.utils.Pose2dWithRotation;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start2Shoot4DynamicAction extends AllegroAutoModeAction {

    public enum FinishStrategy {
        Stop,
        NearSide,
        FarSide
    }

    private static final double kDelayForIntakeDownTime = 0.4 ;
    private static final double kDistToShoot = 0.50 ;

    private static final double [] kPathMaxVelocity = { 3.0, 3.0, 1.25, 1.5, 1.25, 1.5, 4.0 } ;
    private static final double [] kPathMaxAccel = { 2.0, 2.0, 1.0, 1.5, 1.0, 1.5, 3.5 } ;

    private static final Pose2dWithRotation kShootPoseConst = new Pose2dWithRotation(new Pose2d(1.50, 5.55, Rotation2d.fromDegrees(180.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kCollect1PoseConst = new Pose2dWithRotation(new Pose2d(2.40, 5.55, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kBlueCollect2PoseConst = new Pose2dWithRotation(new Pose2d(2.50, 6.32, Rotation2d.fromDegrees(45.0)), Rotation2d.fromDegrees(45.0)) ;
    private static final Pose2dWithRotation kBlueCollect3PoseConst = new Pose2dWithRotation(new Pose2d(2.30, 4.55, Rotation2d.fromDegrees(-45.0)), Rotation2d.fromDegrees(-45.0)) ;
    private static final Pose2dWithRotation kRedCollect2PoseConst = new Pose2dWithRotation(new Pose2d(2.50, 6.72, Rotation2d.fromDegrees(45.0)), Rotation2d.fromDegrees(45.0)) ;
    private static final Pose2dWithRotation kRedCollect3PoseConst = new Pose2dWithRotation(new Pose2d(2.25, 4.9, Rotation2d.fromDegrees(-45.0)), Rotation2d.fromDegrees(-45.0)) ;
    private static final Pose2dWithRotation kNearSidePoseConst = new Pose2dWithRotation(new Pose2d(7.42, 7.37, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kFarSidePose1Const = new Pose2dWithRotation(new Pose2d(2.84, 2.63, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kFarSidePose2Const = new Pose2dWithRotation(new Pose2d(7.42, 0.86, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;

    private enum State {
        Idle,
        Shoot1,
        DelayForIntakeDown,
        DrivingPath1,
        DrivingPath2,
        DrivingPath3,
        DrivingPath4,
        DrivingPath5,
        DrivingPath6,
        DrivingPath7
    }

    private State state_ ;
    private double state_start_time_ ;
    private FinishStrategy finish_ ;

    private Pose2dWithRotation kShootPose ;
    private Pose2dWithRotation kCollect1Pose ;
    private Pose2dWithRotation kCollect2Pose ;
    private Pose2dWithRotation kCollect3Pose ;
    private Pose2dWithRotation kNearSidePose ;
    private Pose2dWithRotation kFarSidePose1 ;
    private Pose2dWithRotation kFarSidePose2 ;

    public Start2Shoot4DynamicAction(AllegroRobot2024 robot, boolean mirror, double mvalue, FinishStrategy finish) throws Exception {
        super(robot, mirror, mvalue) ;
        state_ = State.Idle ;
        state_start_time_ = 0.0 ;
        finish_ = finish ;

        kShootPose = adjustPoseRedBlue(kShootPoseConst) ;
        kCollect1Pose = adjustPoseRedBlue(kCollect1PoseConst) ;
        if (!mirror) {
            kCollect2Pose = adjustPoseRedBlue(kBlueCollect2PoseConst) ;
            kCollect3Pose = adjustPoseRedBlue(kBlueCollect3PoseConst) ;
        }
        else {
            kCollect2Pose = adjustPoseRedBlue(kRedCollect2PoseConst) ;
            kCollect3Pose = adjustPoseRedBlue(kRedCollect3PoseConst) ;
        }
        kNearSidePose = adjustPoseRedBlue(kNearSidePoseConst) ;
        kFarSidePose1 = adjustPoseRedBlue(kFarSidePose1Const) ;
        kFarSidePose2 = adjustPoseRedBlue(kFarSidePose2Const) ;        
    }

    @Override
    public void start() throws Exception {
        super.start();

        // getRobot().getPlotManager().enable("S2S4D-P1") ;
        // getRobot().getPlotManager().enable("S2S4D-P2") ;
        // getRobot().getPlotManager().enable("S2S4D-P3") ;               
        // getRobot().getPlotManager().enable("S2S4D-P4") ;
        // getRobot().getPlotManager().enable("S2S4D-P5") ;
        // getRobot().getPlotManager().enable("S2S4D-P6") ;
        // getRobot().getPlotManager().enable("S2S4D-P7") ;

        state_ = State.Shoot1 ;
        state_start_time_ = getRobot().getTime() ;
        getRobotSubsystem().getIntakeShooter().setAction(getManualShootLowAction(), true) ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        State prev = state_ ;
        switch(state_) {
            case Shoot1:
                if (!getRobotSubsystem().getIntakeShooter().isHoldingNote()) {
                    getRobotSubsystem().getIntakeShooter().setAction(getStartCollectAction(), true) ;
                    state_ = State.DelayForIntakeDown ;                    
                }
                break ;

            case DelayForIntakeDown:
                if (getRobot().getTime() - state_start_time_ > kDelayForIntakeDownTime) {
                    gotoPoseWithRotationAndCollect("S2S4D-P1", kPathMaxVelocity[0], kPathMaxAccel[0], 0.0, 0.0, kCollect1Pose);
                    state_ = State.DrivingPath1 ;
                }
                break ;

            case DrivingPath1:
                if (isCurrentPathDone()) {
                    if (hasNote()) {
                        gotoPoseWithRotationAndShoot("S2S4D-P2", kPathMaxVelocity[1], kPathMaxAccel[1], 0.0, 0.0, kShootPose, kShootPose.getTranslation(), kDistToShoot);
                        state_ = State.DrivingPath2 ; 
                    }
                    else {
                        gotoPoseWithRotationAndCollect("S2S4D-P3", kPathMaxVelocity[2], kPathMaxAccel[2], 0.0, 1.0, kCollect2Pose);
                        state_ = State.DrivingPath3 ;
                    }                    
                }
                break ;
            
            case DrivingPath2:
                if (!hasNote()) {
                    gotoPoseWithRotationAndCollect("S2S4D-P3", kPathMaxVelocity[2], kPathMaxAccel[2], 0.0, 1.0, kCollect2Pose);
                    state_ = State.DrivingPath3 ;
                }
                break ;

            case DrivingPath3:
                if (isCurrentPathDone()) {
                    if (hasNote()) {
                        gotoPoseWithRotationAndShoot("S2S4D-P4", kPathMaxVelocity[3], kPathMaxAccel[3], 0.0, 0.5, kShootPose, kShootPose.getTranslation(), kDistToShoot);
                        state_ = State.DrivingPath4 ;
                        MessageLogger logger = getMessageLogger() ;
                        logger.startMessage(MessageType.Info) ;
                        logger.add("driving path 3 - found note") ;
                        logger.endMessage();
                    }
                    else {
                        // Pose2dWithRotation pts[] = new Pose2dWithRotation[] { kImd2_3, kCollect3Pose} ;                      
                        gotoPoseWithRotationAndCollect("S2S4D-P5", kPathMaxVelocity[4], kPathMaxAccel[4], 0.0, 0.5, kCollect3Pose);
                        state_ = State.DrivingPath5 ;

                        MessageLogger logger = getMessageLogger() ;
                        logger.startMessage(MessageType.Info) ;
                        logger.add("driving path 3 - missed note") ;
                        logger.endMessage();
                    }
                }
                break ;

            case DrivingPath4:
                if (isCurrentPathDone() && !hasNote()) {
                    gotoPoseWithRotationAndCollect("S2S4D-P5", kPathMaxVelocity[4], kPathMaxAccel[4], 0.0, 1.0, kCollect3Pose);
                    state_ = State.DrivingPath5 ;
                }
                break ;

            case DrivingPath5:
                if (isCurrentPathDone()) {
                    gotoPoseWithRotationAndShoot("S2S4D-P6", kPathMaxVelocity[5], kPathMaxAccel[5], 0.0, 0.75, kShootPose, kShootPose.getTranslation(), kDistToShoot);
                    state_ = State.DrivingPath6 ;
                }
                break ;

            case DrivingPath6:
                if (isCurrentPathDone()) {
                    getRobotSubsystem().getSwerve().setAction(getStowAction(), true) ;
                    switch(finish_) {
                        case Stop:
                            break ;

                        case NearSide:
                            gotoPoseWithRotationAndCollect("S2S4D-P7", kPathMaxVelocity[6], kPathMaxAccel[6], 0.0, 0.0, kNearSidePose);
                            break ;

                        case FarSide:
                            gotoPoseWithRotation("S2S4D-P7", kPathMaxVelocity[6], kPathMaxAccel[6], 0.0, 0.0, new Pose2dWithRotation[] { kFarSidePose1, kFarSidePose2 });
                            break ;
                    }
                    state_ = State.Idle ;
                }

            default:
                break ;
        }

        if (prev != state_) {
            state_start_time_ = getRobot().getTime() ;            
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info) ;
            logger.add("Start2Shoot4DynamicAction : " + prev.toString() + " -> " + state_.toString()) ;
            logger.endMessage();
        }        
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "Start2Shoot4DynamicAction" ;
    }
}
