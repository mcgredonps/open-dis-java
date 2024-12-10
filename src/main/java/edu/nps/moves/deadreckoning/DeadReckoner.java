package edu.nps.moves.deadreckoning;

import edu.nps.moves.dis.EntityStatePdu;

/**
 * This Java implementation is based on the C implementation from the paper:
 * Towers, J., and Hines, J., "Equations of motion of the DIS 2.0.3 dead reckoning algorithms,"
 * Tenth Workshop on Standards for the Interoperability of Defense Simulations, Orlando, FL, pp. 431-462, Mar. 1994.
 * Available at http://www.sisostds.org/DigitalLibrary.aspx?EntryId=31599
 *
 * @author camejia
 * @author Alessandro Martinelli
 * @author Alessio Matricardi
 */
public class DeadReckoner {

    private static final IDeadReckoner<EntityStatePdu> dis6DeadReckoner = new Dis6DeadReckoner();

    private static final IDeadReckoner<edu.nps.moves.dis7.EntityStatePdu> dis7DeadReckoner = new Dis7DeadReckoner();

    /**
     * Performs Dead Reckoning using any algorithm 1 through 9.
     * Modifies input pESPDU.
     *
     * @param pESPDU
     * @param deltaTime
     */
    public static void perform_DR(EntityStatePdu pESPDU, double deltaTime) {
        dis6DeadReckoner.performDeadReckoning(pESPDU, deltaTime);
    }

    /**
     * Performs Dead Reckoning for DISv7 entities using any algorithm 1 through 9.
     * Modifies input pESPDU.
     *
     * @param pESPDU
     * @param deltaTime
     */
    public static void perform_DRv7(edu.nps.moves.dis7.EntityStatePdu pESPDU, double deltaTime) {
        dis7DeadReckoner.performDeadReckoning(pESPDU, deltaTime);
    }

}
