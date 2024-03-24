package org.xero1425.base.subsystems.swerve;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.IMotorController.XeroNeutralMode;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MinMaxData;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SDSSwerveDriveSubsystem extends SwerveBaseSubsystem {

    static final double kOneDegreeInRadians = 0.01745329251994329576923690768489 ;

    enum Mode {
        RawPower,
        RawSpeed,
        Chassis
    } ;

    private class Module {
        public final SwerveModule hw ;
        public final PIDCtrl pid ;

        public Module(SwerveModule module, PIDCtrl pidctrl) {
            this.hw = module ;
            this.pid = pidctrl ;
        }
    }

    private Module[] modules_ = new Module[4] ;

    private ChassisSpeeds chassis_speed_ ;
    private boolean disabled_init_ ;
    private boolean coast_mode_ ;

    private Mode mode_ ;
    private double [] speeds_ ;
    private double [] powers_ ;
    private double [] angles_ ;
    private MinMaxData[] history_ ;
    private double stopped_threshold_ ;

    private int module_pos_ = 0 ;

    public SDSSwerveDriveSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name) ;

        disabled_init_ = false ;
	    coast_mode_ = false ;
        stopped_threshold_ = getSettingsValue("props:stopped-threshold").getDouble() ;

        speeds_ = new double[4] ;
        powers_ = new double[4] ;
        angles_ = new double[4] ;
        history_ = new MinMaxData[4] ;
        history_[FL] = new MinMaxData(10) ;
        history_[FR] = new MinMaxData(10) ;
        history_[BL] = new MinMaxData(10) ;
        history_[BR] = new MinMaxData(10) ;

        String mtype = getSettingsValue("hw:modules:type").getString() ;
        String mratio = getSettingsValue("hw:modules:ratio").getString() ;
        SwerveModuleConfig cfg = getConfiguration(mtype, mratio) ;
        if (cfg == null) {
            throw new Exception("Swerve module: type = '" + mtype + "', ratio = '" + mratio + "' not supported") ;
        }
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        modules_[FL] = createModule(cfg, "fl", shuffleboardTab) ;
        modules_[FR] = createModule(cfg, "fr", shuffleboardTab) ;
        modules_[BL] = createModule(cfg, "bl", shuffleboardTab) ;
        modules_[BR] = createModule(cfg, "br", shuffleboardTab) ;
        
        createOdometry();
    }

    @Override
    public void postHWInit() {
        try {
            //
            // We do this here instead of in the constructor so that the
            // simulation model has had a chance to run for the swerve modules and have
            // initialized the cancoder to the right value
            //
            modules_[FL].hw.synchronizeEncoders(true);
            modules_[FR].hw.synchronizeEncoders(true);
            modules_[BL].hw.synchronizeEncoders(true);
            modules_[BR].hw.synchronizeEncoders(true);            
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("exception thrown while synchronizing angle in swerve modules").endMessage();
            logger.logStackTrace(ex.getStackTrace());
        }
    }

    public SwerveModule getModule(int which) {
        return modules_[which].hw ;
    }

    SwerveModuleConfig getConfiguration(String mtype, String mratio) {
        SwerveModuleConfig cfg = null ;

        if (mtype.toLowerCase().equals("mk4i")) {
            if (mratio.toLowerCase().equals("l3")) {
                cfg = SwerveModuleConfig.MK4I_L3 ;
            }
            else if (mratio.toLowerCase().equals("l2")) {
                cfg = SwerveModuleConfig.MK4I_L2 ;
            }     
            else if (mratio.toLowerCase().equals("l1")) {
                cfg = SwerveModuleConfig.MK4I_L1 ;
            }                      
        }

        return cfg ;
    }

    private Module createModule(SwerveModuleConfig cfg, String which, ShuffleboardTab tab) throws Exception {
        String basename = "subsystems:" + getName() + ":hw:" ;

        ShuffleboardLayout lay = tab.getLayout(which, BuiltInLayouts.kList).withSize(2, 3).withPosition(module_pos_, 0) ;
        module_pos_ += 2 ;
        SwerveModule hw = new SwerveModule(this, cfg, lay, which, basename + which);
        PIDCtrl pid = createPIDCtrl(which) ;
        return new Module(hw, pid) ;
    }

    private PIDCtrl createPIDCtrl(String name) throws MissingParameterException, BadParameterTypeException {
        String pidname = "subsystems:" + getName() + ":pids:" + name ;
        return new PIDCtrl(getRobot().getSettingsSupplier(), pidname, false) ;
    }    

    public SwerveModuleState getModuleState(int which) throws BadMotorRequestException, MotorRequestFailedException {
        SwerveModuleState st = null ;

        switch(which) {
            case FL:
                st = new SwerveModuleState(modules_[FL].hw.getDriveVelocity(), new Rotation2d(modules_[FL].hw.getStateAngle())) ;
                break ;

            case FR:
                st = new SwerveModuleState(modules_[FR].hw.getDriveVelocity(), new Rotation2d(modules_[FR].hw.getStateAngle())) ;
                break ;
                
            case BL:
                st = new SwerveModuleState(modules_[BL].hw.getDriveVelocity(), new Rotation2d(modules_[BL].hw.getStateAngle())) ;
                break ;
                
            case BR:
                st = new SwerveModuleState(modules_[BR].hw.getDriveVelocity(), new Rotation2d(modules_[BR].hw.getStateAngle())) ;
                break ;
        }

        return st ;
    }

    public SwerveModulePosition getModulePosition(int which) {
        SwerveModulePosition st = null ;

        try {
            switch(which) {
                case FL:
                    st = new SwerveModulePosition(modules_[FL].hw.getDistance(), new Rotation2d(modules_[FL].hw.getStateAngle())) ;
                    break ;

                case FR:
                    st = new SwerveModulePosition(modules_[FR].hw.getDistance(), new Rotation2d(modules_[FR].hw.getStateAngle())) ;
                    break ;
                    
                case BL:
                    st = new SwerveModulePosition(modules_[BL].hw.getDistance(), new Rotation2d(modules_[BL].hw.getStateAngle())) ;
                    break ;
                    
                case BR:
                    st = new SwerveModulePosition(modules_[BR].hw.getDistance(), new Rotation2d(modules_[BR].hw.getStateAngle())) ;
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

        //
        // Note: These lines will on perform a syncronization on a module if it has
        //       not been synchronized.  The modules should have been sync'ed when they
        //       were created.  This is just an out in case an error occurred during the
        //       sync process during creation.
        //
        modules_[FL].hw.synchronizeEncoders(false);
        modules_[FR].hw.synchronizeEncoders(false);
        modules_[BL].hw.synchronizeEncoders(false);
        modules_[BR].hw.synchronizeEncoders(false);                        

        if (getRobot().isDisabled()) {
            if (!disabled_init_) {
                modules_[FL].hw.setNeutralMode(XeroNeutralMode.Coast);
                modules_[FR].hw.setNeutralMode(XeroNeutralMode.Coast);
                modules_[BL].hw.setNeutralMode(XeroNeutralMode.Coast);
                modules_[BR].hw.setNeutralMode(XeroNeutralMode.Coast);      
                coast_mode_ = true ;                              

                modules_[FL].hw.set(FL, 0.0, modules_[FL].hw.getStateAngle());
                modules_[FR].hw.set(FR, 0.0, modules_[FR].hw.getStateAngle());
                modules_[BL].hw.set(BL, 0.0, modules_[BL].hw.getStateAngle());
                modules_[BR].hw.set(BR, 0.0, modules_[BR].hw.getStateAngle());

                disabled_init_ = true ;
            }
        }
        else {
            if (coast_mode_) {
                modules_[FL].hw.setNeutralMode(XeroNeutralMode.Brake);
                modules_[FR].hw.setNeutralMode(XeroNeutralMode.Brake);
                modules_[BL].hw.setNeutralMode(XeroNeutralMode.Brake);
                modules_[BR].hw.setNeutralMode(XeroNeutralMode.Brake);
                coast_mode_ = false ;            
            }
            disabled_init_ = false ;
        }
    }

    private boolean allModulesSynchronized() {
        return modules_[FL].hw.isEncoderSynchronized() && modules_[FR].hw.isEncoderSynchronized() &&
                modules_[BL].hw.isEncoderSynchronized() && modules_[BR].hw.isEncoderSynchronized() ;
    }

    @Override
    public boolean isStopped() {
        return history_[FL].getMax() < stopped_threshold_ && history_[FR].getMax() < stopped_threshold_ &&
                history_[BL].getMax() < stopped_threshold_ && history_[BR].getMax() < stopped_threshold_ ;
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
            powers_[FL] = modules_[FL].pid.getOutput(speeds_[FL], getModuleState(FL).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[FR] = modules_[FR].pid.getOutput(speeds_[FR], getModuleState(FR).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[BL] = modules_[BL].pid.getOutput(speeds_[BL], getModuleState(BL).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[BR] = modules_[BR].pid.getOutput(speeds_[BR], getModuleState(BR).speedMetersPerSecond, getRobot().getDeltaTime()) ;
        }

        if (allModulesSynchronized()) {            
            modules_[FL].hw.set(FL, powers_[FL], Math.toRadians(angles_[FL])) ;
            modules_[FR].hw.set(FR, powers_[FR], Math.toRadians(angles_[FR])) ;
            modules_[BL].hw.set(BL, powers_[BL], Math.toRadians(angles_[BL])) ;
            modules_[BR].hw.set(BR, powers_[BR], Math.toRadians(angles_[BR])) ;

            history_[FL].addData(powers_[FL]);
            history_[FR].addData(powers_[FR]);
            history_[BL].addData(powers_[BL]);
            history_[BR].addData(powers_[BR]);
        }
        else {
            MessageLogger logger = getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info);
            logger.add("SDSSwerveDriveSubsystem: waiting on timer to init angles from encoders") ;
            logger.endMessage();
        }
    }
}
