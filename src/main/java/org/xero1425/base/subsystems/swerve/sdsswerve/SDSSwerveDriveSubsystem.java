package org.xero1425.base.subsystems.swerve.sdsswerve;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorFactory;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.PIDCtrl;
import org.xero1425.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SDSSwerveDriveSubsystem extends SwerveBaseSubsystem {

    enum Mode {
        RawPower,
        RawSpeed,
        Chassis
    } ;

    private final SwerveModule fl_ ;
    private final SwerveModule fr_ ;
    private final SwerveModule bl_ ;
    private final SwerveModule br_ ;

    private PIDCtrl[] pid_ctrls_ ;

    private ChassisSpeeds chassis_speed_ ;

    private Mode mode_ ;
    private double [] speeds_ ;
    private double [] powers_ ;
    private double [] angles_ ;

    private double nominal_voltage_ ;

    private boolean module_encoders_inited_ ;
    
    public SDSSwerveDriveSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name) ;

        speeds_ = new double[4] ;
        powers_ = new double[4] ;
        angles_ = new double[4] ;

        String basename = "subsystems:" + name + ":hw:" ;
        MotorFactory factory = parent.getRobot().getMotorFactory() ;
        ISettingsSupplier settings = parent.getRobot().getSettingsSupplier() ;
        fl_ = new SwerveModule(factory, settings, "swerve-fl", basename + "fl");
        fr_ = new SwerveModule(factory, settings, "swerve-fr", basename + "fr");
        bl_ = new SwerveModule(factory, settings, "swerve-bl", basename + "bl");        
        br_ = new SwerveModule(factory, settings, "swerve-br", basename + "br");
    }

    public SwerveModuleState getModuleState(int which) throws BadMotorRequestException, MotorRequestFailedException {
        SwerveModuleState st = null ;

        switch(which) {
            case FL:
                st = new SwerveModuleState(fl_.getDriveVelocity(), new Rotation2d(fl_.getSteerAngle())) ;
                break ;

            case FR:
                st = new SwerveModuleState(fr_.getDriveVelocity(), new Rotation2d(fr_.getSteerAngle())) ;
                break ;
                
            case BL:
                st = new SwerveModuleState(bl_.getDriveVelocity(), new Rotation2d(bl_.getSteerAngle())) ;
                break ;
                
            case BR:
                st = new SwerveModuleState(br_.getDriveVelocity(), new Rotation2d(br_.getSteerAngle())) ;
                break ;
        }

        return st ;
    }

    public SwerveModulePosition getModulePosition(int which) {
        SwerveModulePosition st = null ;

        try {
            switch(which) {
                case FL:
                    st = new SwerveModulePosition(fl_.getDistance(), new Rotation2d(fl_.getSteerAngle())) ;
                    break ;

                case FR:
                    st = new SwerveModulePosition(fr_.getDistance(), new Rotation2d(fr_.getSteerAngle())) ;
                    break ;
                    
                case BL:
                    st = new SwerveModulePosition(bl_.getDistance(), new Rotation2d(bl_.getSteerAngle())) ;
                    break ;
                    
                case BR:
                    st = new SwerveModulePosition(br_.getDistance(), new Rotation2d(br_.getSteerAngle())) ;
                    break ;
            }
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("exception thrown by SDSSwerveDriveSubsystem.getModulePosition - " + ex.getMessage()).endMessage();
            logger.logStackTrace(ex.getStackTrace());
        }

        return st ;
    }

    public SwerveModuleState getModuleTarget(int which) {
        SwerveModuleState st = null ;

        st = new SwerveModuleState(speeds_[which], Rotation2d.fromDegrees(angles_[which])) ;
        return st ;
    }


    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(gyro().getYaw()) ;
    }

    public void drive(ChassisSpeeds speed) {
        chassis_speed_ = speed ;     
        mode_ = Mode.Chassis ;
    }


    @Override
    public void setRawTargets(boolean power, double [] angles, double [] speeds_powers)  {
        angles_ = angles.clone() ;
        if (power) {
            mode_ = Mode.RawPower ;
            powers_ = speeds_powers.clone() ;
        }
        else {
            mode_ = Mode.RawSpeed ;
            speeds_ = speeds_powers.clone() ;
        }
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        if (getRobot().isDisabled()) {
            fl_.set(0.0, fl_.getSteerAngle());
            fr_.set(0.0, fr_.getSteerAngle());
            bl_.set(0.0, bl_.getSteerAngle());
            br_.set(0.0, br_.getSteerAngle());
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        //
        // Just in case, paranoid code.  Be sure the arrays we are intersted in are there.  SHould be set
        // in the constructor and in any setter that provides new values
        //
        if (angles_ == null || angles_.length != 4)
            angles_ = new double[4] ;

        if (speeds_ == null || speeds_.length != 4)
            speeds_ = new double[4] ;

        if (powers_ == null || powers_.length != 4)
            powers_ = new double[4] ;   

        if (mode_ == Mode.Chassis) {

            // Convert chassis speeds to module speeds and angles
            SwerveModuleState[] states = getKinematics().toSwerveModuleStates(chassis_speed_);
            
            angles_[FL] = states[FL].angle.getDegrees() ;
            speeds_[FL] = states[FL].speedMetersPerSecond ;
            angles_[FR] = states[FR].angle.getDegrees() ;
            speeds_[FR] = states[FR].speedMetersPerSecond ;
            angles_[BL] = states[BL].angle.getDegrees() ;
            speeds_[BL] = states[BL].speedMetersPerSecond ;
            angles_[BR] = states[BR].angle.getDegrees() ;
            speeds_[BR] = states[BR].speedMetersPerSecond ;                                    
        }

        if (mode_ == Mode.Chassis || mode_ == Mode.RawSpeed)
        {
            powers_[FL] = pid_ctrls_[FL].getOutput(speeds_[FL], getModuleState(FL).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[FR] = pid_ctrls_[FR].getOutput(speeds_[FR], getModuleState(FR).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[BL] = pid_ctrls_[BL].getOutput(speeds_[BL], getModuleState(BL).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[BR] = pid_ctrls_[BR].getOutput(speeds_[BR], getModuleState(BR).speedMetersPerSecond, getRobot().getDeltaTime()) ;
        }

        if (module_encoders_inited_) {
            fl_.set(powers_[FL] * nominal_voltage_, Math.toRadians(angles_[FL])) ;
            fr_.set(powers_[FR] * nominal_voltage_, Math.toRadians(angles_[FR])) ;
            bl_.set(powers_[BL] * nominal_voltage_, Math.toRadians(angles_[BL])) ;
            br_.set(powers_[BR] * nominal_voltage_, Math.toRadians(angles_[BR])) ;

            // MessageLogger logger = getRobot().getMessageLogger();
            // logger.startMessage(MessageType.Info);
            // logger.add("swerve driving: ") ;
            // logger.add("fl", angles_[FL]);
            // logger.add("fr", angles_[FR]);
            // logger.add("bl", angles_[BL]);
            // logger.add("br", angles_[BR]);
            // logger.endMessage();
        }
        else {
            MessageLogger logger = getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info);
            logger.add("SDSSwerveDriveSubsystem: waiting on timer to init angles from encoders") ;
            logger.endMessage();
        }
    }
}
