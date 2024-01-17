package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.SwerveTestAutoMode;

public class AllegroTestAutoMode extends SwerveTestAutoMode {

    public AllegroTestAutoMode(AutoController ctrl) throws Exception {
        super(ctrl, "Allegro-Test-Mode");

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
            case 10:
                
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
