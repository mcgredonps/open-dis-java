package edu.nps.moves.deadreckoning;

/**
 * IDeadReckoner
 * <br>
 * Provides the signature of Dead Reckoning algorithm
 *
 * @author Alessio Matricardi
 */
public interface IDeadReckoner<T> {

    /**
     * Perform the Dead Reckoning algorithm on {@code entityStatePdu}
     *
     * @param entityStatePdu the PDU
     * @param deltaTime      the time increment for the dead reckoning step
     */
    void performDeadReckoning(T entityStatePdu, double deltaTime);

}
