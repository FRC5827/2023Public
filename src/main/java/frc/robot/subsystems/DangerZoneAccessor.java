package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Helper class for making access to a DangerZone easy. It is using upper/lower bound values to define a danger zone.
 */
public final class DangerZoneAccessor {
    /**
     * The owner of this accessor.
     */
    private final SubsystemBase m_owner;

    /**
     * The danger zone that needs to be entered/exited.
     */
    private final DangerZone m_dangerZone;

    /**
     * The number lower bound of the danger zone.
     */
    private final double m_lowerBound;

    /**
     * The number upper bound of the danger zone.
     */
    private final double m_upperBound;

    /**
     * The releaser for the danger zone. If this is not null, then it means the owner has entered the danger zone.
     */
    private DangerZone.Releaser releaser;

    /**
     *  The danger zone resides between lower and upperbounds. If a value is between those two, then the system is in the danger zone.
     * @param owner the owner of this DangerZone accessor.
     * @param dangerZone the danger zone to go into.
     * @param lowerBound the lower bound of the danger zone.
     * @param upperBound the upper bound of the danger zone.
     */
    public DangerZoneAccessor(SubsystemBase owner, DangerZone dangerZone, double lowerBound, double upperBound) {
        this.m_owner = owner;
        this.m_dangerZone = dangerZone;
        this.m_lowerBound = lowerBound;
        this.m_upperBound = upperBound;
    }

    /**
     * Check whether the value is in danger and try to enter the zone (or stay in it) if it is. 
     * 
     * If this returns true, then it is safe to keep doing the work. If it returns false then there is a risk of
     * entering the danger zone but it is occuppied and the caller should pause the operation until the danger zone is vacated.
     */
    public boolean checkAndTryEnter(double value) {
        if (this.releaser != null) {            
            
            // We're already in danger zone.

            if (!this.IsInDangerBounds(value)) {
                // With the new value, we left the danger zone. release the zone for others.
                DangerZone.Releaser releaser = this.releaser;
                this.releaser = null;
                releaser.release();
            }

            return true;
        }
        else {

            // We're not in the danger zone already

            if (this.IsInDangerBounds(value)) {
                // We're about to enter the danger zone.
                DangerZone.Releaser releaser = this.m_dangerZone.tryEnter(this.m_owner);
                if (releaser == null) {
                    // We couldn't enter the danger zone
                    return false;
                }

                // We entered the danger zone.
                this.releaser = releaser;
                return true;
            }

            // We're not going into the danger zone.
            return true;
        }
    }

    private boolean IsInDangerBounds(double value) {
        return value < this.m_upperBound && value > this.m_lowerBound;
    }
}