package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * This class represents the critical zone where subsystems can collide (e.g. the arm and the intake).
 * If a component enters a danger zone, the others must wait until the entrant vacates the
 * zone.
 */
public final class DangerZone {
    /** 
     * lock that protects the access to the danger zone.
     */     
    private final Object m_lock = new Object(); 

    /**
     * The current occupent of the danger zone, or null.
     */
    private SubsystemBase m_occupant;
    
    /**
     * A nonce that makes sure that we don't accidentally release the danger zone from the wrong subsystem.
     */
    private int m_lockId = 0;

    /**
     * Try to enter the danger zone. If there is not an occupant, then the component can 
     * enter and the method returns a releaser that can be used to signal that the new occupant has vacated the dnager zone..
     * If there is an existing occuppant, then it cannot enter the danger zone, and must stop and wait until it can.
     * @return
     */
    public Releaser tryEnter(SubsystemBase candidate) {

        synchronized (this.m_lock)
        {
            if (this.m_occupant != null) {
                System.out.println(
                    "Candidate: '" + candidate.getName() +"' cannot enter the danger zone. Occupant '" + this.m_occupant.getName() + "' is already in the danger zone.");
                    return null;
            }

            this.m_occupant = candidate;
            this.m_lockId++;
            this.m_lockId = this.m_lockId % (Integer.MAX_VALUE - 1); // Handle wraparound and get non-negative numbers.

            System.out.println("Subsystem is entering danger zone: " + this.m_occupant.getName() + " with lock id: " + this.m_lockId);
            SmartDashboard.putString("Dangerzone occupant", this.m_occupant.getName());
            SmartDashboard.putNumber("Dangerzone lockid", this.m_lockId);
            return new Releaser(this.m_lockId);
        }
    }

    private void release(long lockId) {

        synchronized (this.m_lock) {
            if (this.m_lockId != lockId) {
                System.out.println("Unable to release for passed in lockId: " + lockId + ". Current lock id: " + this.m_lockId);
                return;
            }
            if (this.m_occupant == null) {
                System.out.println("Double releasing danger zone for lock id: " + lockId);
                return;
            }

            this.m_occupant = null;
            SmartDashboard.putString("Dangerzone occupant", "<empty>");
            SmartDashboard.putNumber("Dangerzone lockid", -1);
        }
    }

    public final class Releaser
    {
        private final long lockId;

        private Releaser(long lockId) {
            this.lockId = lockId;
        }

        public void release()
        {
            DangerZone.this.release(this.lockId);
        }
    }
}
