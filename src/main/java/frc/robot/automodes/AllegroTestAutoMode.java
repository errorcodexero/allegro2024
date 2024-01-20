package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.SwerveTestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class AllegroTestAutoMode extends SwerveTestAutoMode {

    public AllegroTestAutoMode(AutoController ctrl) throws Exception {
        super(ctrl, "Allegro-Test-Mode");

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        IntakeShooterSubsystem intake = robot.getIntakeShooterSubsystem() ;

        if (createTest()) {
            //
            // If this returns true, the test mode was created for the swerve drive related tests
            //
            return ;
        }

        switch (getTestNumber()) {
            //////////////////////////////////////////////////////////////////////////////////////
            //
            // Intake test modes
            //
            //////////////////////////////////////////////////////////////////////////////////////

            // Run the spinner motor
            case 10:
                if (intake != null) {
                    addSubActionPair(intake.spinner(), new MotorEncoderPowerAction(intake.spinner(), getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            //////////////////////////////////////////////////////////////////////////////////////
            //
            // Shooter test modes
            //
            //////////////////////////////////////////////////////////////////////////////////////
            case 20:
                break ;

            //////////////////////////////////////////////////////////////////////////////////////
            //
            // Elevator/Arm test modes
            //
            //////////////////////////////////////////////////////////////////////////////////////
            case 30:
                break ;

            //////////////////////////////////////////////////////////////////////////////////////
            //
            // Dog Head testmodes
            //
            //////////////////////////////////////////////////////////////////////////////////////
            case 40:
                break ; 
        }
    }
}
