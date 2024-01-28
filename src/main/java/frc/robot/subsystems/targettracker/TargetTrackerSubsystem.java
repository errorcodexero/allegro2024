package frc.robot.subsystems.targettracker;

import java.util.Optional;

import org.xero1425.base.IVisionAlignmentData.CamMode;
import org.xero1425.base.IVisionAlignmentData.LedMode;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.swerve.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TargetTrackerSubsystem extends Subsystem {
    private int speaker_tag_  ;

    private SwerveBaseSubsystem db_ ;
    private LimeLightSubsystem ll_ ;
    private Pose2d speaker_loc_ ;
    private boolean ll_has_speaker_ ;
    private double distance_ ;
    private double angle_ ;

    public TargetTrackerSubsystem(Subsystem parent, SwerveBaseSubsystem db, LimeLightSubsystem ll) {
        super(parent, "TargetTracker") ;

        db_ = db ;
        ll_ = ll ;

        ll_.setCamMode(CamMode.VisionProcessing);
        ll_.setLedMode(LedMode.ForceOff);
        ll_.setPipeline(0);

        speaker_tag_ = -1 ;
        speaker_loc_ = null ;
        ll_has_speaker_ = false ;
    }

    public double getDistanceToSpeaker() {
        return distance_ ;
    }

    public double getAngleToSpeaker() {
        return angle_ ;
    }

    public boolean hasSpeakerLocation() {
        return speaker_tag_ != -1 ;
    }

    public boolean llHasSpeaker() {
        return ll_has_speaker_ ;
    }

    @Override
    public void computeMyState() {
        if (speaker_tag_ == -1) {
            if (!getSpeakerTag())
                return ;
        }

        if (ll_.hasAprilTag(speaker_tag_)) {
            //
            // If we see the april tag, we can setup to shoot
            //
            computeAngleDistanceFromLimeLight() ;
        }
        else {
            //
            // If we don't see the april tag, we can't setup to shoot.  So we just
            // provide the angle to rotate to the drive base.  The drive base can
            // decide if it wants to rotate or not.
            //
            computeAngleDistanceFromDriveBasePose() ;
        }

        db_.setRotationAngle(angle_);
    }

    @Override
    public void run() {
    }

    private void computeAngleDistanceFromLimeLight() {
        ll_has_speaker_ = true ;

        LimeLightSubsystem.DistanceAngle da = ll_.getDistanceAngleToTag(speaker_tag_) ;
        distance_ = da.distance ;
        angle_ = da.angle ;
    }

    private void computeAngleDistanceFromDriveBasePose() {
        ll_has_speaker_ = false ;

        Translation2d dbloc = db_.getPose().getTranslation() ;
        Translation2d v = speaker_loc_.getTranslation().minus(dbloc);

        distance_ = v.getNorm() ;
        angle_ = v.getAngle().getDegrees() ;
    }

    private boolean getSpeakerTag() {
        boolean ret = false ;
        Optional<Alliance> a = DriverStation.getAlliance() ;
        if (a.isPresent()) {
            speaker_tag_ = (a.get() == Alliance.Blue) ? AprilTagLocations.BlueSpeakerCenter : AprilTagLocations.RedSpeakerCenter ;
            ret = true ;
            Optional<Pose3d> p3d = getRobot().getAprilTags().getTagPose(speaker_tag_) ;
            if (p3d.isPresent()) {
                speaker_loc_ = p3d.get().toPose2d() ;
            }
            else {
                //
                // This should never happen.
                //
                speaker_tag_ = -1 ;
                ret = false ;
            }
        }

        return ret ;
    }
}
