package org.xero1425.base.subsystems.swerve;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public abstract class SwerveHolonomicControllerAction extends SwerveDriveAction {
    private SwerveBaseSubsystem swerve_ ;
    private HolonomicDriveController ctrl_ ;

    public SwerveHolonomicControllerAction(SwerveBaseSubsystem sub) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        swerve_ = sub ;
        ctrl_ = createDriveController("") ;
    }    

    public SwerveHolonomicControllerAction(SwerveBaseSubsystem sub, String suffix) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        swerve_ = sub ;
        ctrl_ = createDriveController(suffix) ;
    }

    protected HolonomicDriveController controller() {
        return ctrl_ ;
    }

    protected HolonomicDriveController createDriveController(String suffix) throws BadParameterTypeException, MissingParameterException {
        HolonomicDriveController ctrl = null ;
        double kp, ki, kd ;

        double maxv = swerve_.getSettingsValue("physical:max-angular-speed").getDouble() ;
        double maxa = swerve_.getSettingsValue("physical:max-angular-accel").getDouble() ;

        String prefix = "path-following" + suffix ;

        kp = swerve_.getSettingsValue(prefix + ":xctrl:kp").getDouble() ;
        ki = swerve_.getSettingsValue(prefix + ":xctrl:ki").getDouble() ;
        kd = swerve_.getSettingsValue(prefix + ":xctrl:kd").getDouble() ;
        PIDController xctrl = new PIDController(kp, ki, kd) ;

        kp = swerve_.getSettingsValue(prefix + ":yctrl:kp").getDouble() ;
        ki = swerve_.getSettingsValue(prefix + ":yctrl:ki").getDouble() ;
        kd = swerve_.getSettingsValue(prefix + ":yctrl:kd").getDouble() ;
        PIDController yctrl = new PIDController(kp, ki, kd) ;

        kp = swerve_.getSettingsValue(prefix + ":rotation:kp").getDouble() ;
        ki = swerve_.getSettingsValue(prefix + ":rotation:ki").getDouble() ;
        kd = swerve_.getSettingsValue(prefix + ":rotation:kd").getDouble() ;
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxv, maxa) ;
        ProfiledPIDController thetactrl = new ProfiledPIDController(kp, ki, kd, constraints) ;
        thetactrl.enableContinuousInput(-Math.PI, Math.PI);
        
        ctrl = new HolonomicDriveController(xctrl, yctrl, thetactrl) ;
        ctrl.setEnabled(true);
        
        double xytol = swerve_.getSettingsValue(prefix + ":xy-tolerance").getDouble() ;
        double angletol = swerve_.getSettingsValue(prefix + "angle-tolerance").getDouble() ;
        ctrl.setTolerance(new Pose2d(xytol, xytol, Rotation2d.fromDegrees(angletol))) ;

        return ctrl ;
    }
}
