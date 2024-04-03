package frc.robot.subsystems.intake_shooter;

public class IntakeSensorThread extends Thread {
    private final static int kSensorThreadLoopInterval = 2 ;
    private Object lock_ ;
    private IntakeShooterSubsystem sub_ ;
    private boolean running_ ;
    private boolean saw_note_ ;

    public IntakeSensorThread(IntakeShooterSubsystem sub) {
        lock_ = new Object() ;
    }

    public void run() {
        while (running_) {
            synchronized(lock_) {
                if (sub_.getSensorRawState()) {
                    saw_note_ = true ;
                }
            }

            try {
                Thread.sleep(kSensorThreadLoopInterval);
            } catch (Exception ex) {
            }            
        }        
    }

    public boolean didSeeNote() {
        boolean ret = false ;

        synchronized(lock_) {
            if (saw_note_) {
                ret = true ;
                saw_note_ = false ;
            }
        }

        return ret ;
    }
}
