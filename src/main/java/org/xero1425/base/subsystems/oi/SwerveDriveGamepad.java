package org.xero1425.base.subsystems.oi;

import java.sql.Driver;
import java.util.Optional;

import org.xero1425.base.LoopType;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.SwerveDriveChassisSpeedAction;
import org.xero1425.base.subsystems.swerve.SwerveDriveXPatternAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SwerveDriveGamepad extends Gamepad {
    private SwerveBaseSubsystem db_;                    // The drive base we are controlling
    private double angle_maximum_;                      // The maximum angle value in degrees
    private double pos_maximum_;                        // The maximum position value in meters
    private double deadband_pos_x_ ;
    private double deadband_pos_y_ ;
    private double deadband_rotate_ ;
    private double power_ ;
    private SwerveDriveChassisSpeedAction action_;
    private SwerveDriveXPatternAction x_action_;
    private double ysign_ = 1.0 ;

    private boolean holding_x_;

    public SwerveDriveGamepad(OISubsystem oi, int index, SwerveBaseSubsystem drive_) throws Exception {
        super(oi, "swerve_gamepad", index);

        if (DriverStation.getStickPOVCount(getIndex()) == 0) {
            throw new Exception("invalid gamepad for SwerveGamepad");
        }

        if (DriverStation.getStickAxisCount(getIndex()) <= AxisNumber.RIGHTX.value) {
            throw new Exception("invalid gamepad for SwerveGamepad");
        }

        db_ = drive_;
        holding_x_ = false ;
    }

    public String getState() {
        return  "power " + power_ + ", rotate:deadband " + deadband_rotate_ + ", pos:deadband:x " + deadband_pos_x_ + ", pos:deadband:y " + deadband_pos_y_ ;
    }

    @Override
    public void init(LoopType ltype) {
        if (ltype == LoopType.Autonomous || ltype == LoopType.Teleop) {
            Optional<Alliance> value = DriverStation.getAlliance() ;
            if (value.isPresent() && value.get() == Alliance.Blue) {
                ysign_ = -1 ;
            }
        }
    }

    @Override
    public void createStaticActions() throws BadParameterTypeException, MissingParameterException {
        deadband_pos_x_ = getSubsystem().getSettingsValue("swerve_gamepad:position:deadband:x").getDouble() ;
        deadband_pos_y_ = getSubsystem().getSettingsValue("swerve_gamepad:position:deadband:y").getDouble() ;
        deadband_rotate_ = getSubsystem().getSettingsValue("swerve_gamepad:angle:deadband").getDouble() ;
        pos_maximum_ = getSubsystem().getSettingsValue("swerve_gamepad:position:maximum").getDouble();
        angle_maximum_ = getSubsystem().getSettingsValue("swerve_gamepad:angle:maximum").getDouble();
        power_ = getSubsystem().getSettingsValue("swerve_gamepad:power").getDouble();

        action_ = new SwerveDriveChassisSpeedAction(db_) ;
        x_action_ = new SwerveDriveXPatternAction(db_);
    }

    public void resetSwerveDriveDirection() {
        RobotSubsystem robotSubsystem = getSubsystem().getRobot().getRobotSubsystem();
        SwerveBaseSubsystem db = (SwerveBaseSubsystem)robotSubsystem.getDB() ;
        if (db != null) {
            boolean invert = false ;
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                invert = true ;
            }
            try {
                db.resetPose(invert);
            }
            catch(Exception ex) {
                MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Error).add("exeception thrown in swerve resetPose - " + ex.getMessage()).endMessage();
                logger.logStackTrace(ex.getStackTrace());
            }
        }
    }

    public void startDriveBaseX() {
        if (db_.getAction() != x_action_) {
            db_.setAction(x_action_);
        }
        action_.update(new ChassisSpeeds());
        holding_x_ = true ;
    }

    public void stopDriveBaseX() {
        if (db_.getAction() != action_ && getSubsystem().getRobot().isTeleop()) {
            db_.setAction(action_);
        }
        holding_x_ = false ;
    }

    @Override
    public void computeState() {
        super.computeState();
    }

    @Override
    public void generateActions() {
        double ly, lx, rx ;

        if (db_ == null || !isEnabled() || holding_x_)
            return ;

        // For X axis, left is -1, right is +1
        // For Y axis, forward is -1, back is +1

        try {
            ly = -DriverStation.getStickAxis(getIndex(), AxisNumber.LEFTY.value) * ysign_ ;
            lx = -DriverStation.getStickAxis(getIndex(), AxisNumber.LEFTX.value) * ysign_ ;
            rx = -DriverStation.getStickAxis(getIndex(), AxisNumber.RIGHTX.value) ;
        }
        catch(Exception ex) {
            return ;
        }
        
        double lyscaled = mapJoyStick(ly, pos_maximum_, deadband_pos_y_, power_) ;
        double lxscaled = mapJoyStick(lx, pos_maximum_, deadband_pos_x_, power_) ;
        double rxscaled = mapJoyStick(rx, angle_maximum_, deadband_rotate_, power_) ;

        if (isLTriggerPressed()) {
            lxscaled *= 0.25 ;
            lyscaled *= 0.25 ;
            rxscaled *= 0.25 ;
        }

        if (Math.abs(rxscaled) < deadband_rotate_ && (Math.abs(lxscaled) > deadband_pos_x_ || Math.abs(lyscaled) > deadband_pos_y_)) {
            //
            // The rotation stick is set to zero, so we want to maintain the current angle.
            //
            if (!db_.getSWRotationControl()) {
                //
                // This is the first robot loop with the rotation stick at zero.  Setup the mode to
                // hold the angle and remember the current robot angle.
                //
                MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
                logger.add("swerve gamepad, locking rotation") ;
                logger.add("rotation", db_.getHeading().getDegrees()) ;
                logger.endMessage();
                db_.setSWRotationControl(true);
                db_.setSWRotationAngle(db_.getHeading().getDegrees());
            }
        }
        else {
            //
            // The rotation stick is not zero, so we want to rotate the robot.  Turn off the angle
            // control mechanism in the drive base and get this data from the joystick.
            //
            db_.setSWRotationControl(false);
        }

        //
        // The rotational velocity is given by rxscaled
        // The position velocity is given by the vector (lyscaled, lxscaled)
        //
        // Note, the x and y are swapped because of the orientation of the gamepad versus the orientation of the
        // field.  The drive team is at the end of the field looking down the X axis.  So, when the Y axis on the
        // gamepad is pushed forward (negative value from the gamepad), the driver expects the robot to move along
        // the positive X axis of the field.
        //
        rxscaled *= 2.0 / Math.hypot(db_.getLength(), db_.getWidth()) / 39.37;  // 39.27 to convert meters -> inches. Original equation from SDS assumes inches.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-lyscaled, -lxscaled, rxscaled, db_.getHeading()) ;
        action_.update(speeds) ;

        if (db_.getAction() != action_)
            db_.setAction(action_) ;
    }

    private double mapJoyStick(double v, double maxv, double db, double power) {
        if (Math.abs(v) < db)
            return 0.0 ;

        return Math.signum(v) * Math.pow(Math.abs(v), power) * maxv ;
    }
}
