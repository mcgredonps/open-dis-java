package edu.nps.moves.deadreckoning;

import edu.nps.moves.dis.ArticulationParameter;
import edu.nps.moves.dis.EntityStatePdu;
import edu.nps.moves.dis7.Orientation;
import edu.nps.moves.dis7.Vector3Double;
import edu.nps.moves.dis7.Vector3Float;

/**
 * Dis6DeadReckoner
 * <br>
 * DIS v6 Dead Reckoning algorithm
 * @author Alessio Matricardi
 */
public final class Dis6DeadReckoner extends AbstractDeadReckoner<EntityStatePdu> {
    @Override
    protected DeadReckoningEntity convertToDeadReckoningEntity(EntityStatePdu entityStatePdu) {
        DeadReckoningEntity toReturn = new DeadReckoningEntity();
        // timestamp
        toReturn.setTimestamp(entityStatePdu.getTimestamp());
        // dead reckoning algorithm
        toReturn.setDeadReckoningAlgorithm(entityStatePdu.getDeadReckoningParameters().getDeadReckoningAlgorithm());
        // entity location
        toReturn.setLocation(new Vector3Double());
        toReturn.getLocation().setX(entityStatePdu.getEntityLocation().getX());
        toReturn.getLocation().setY(entityStatePdu.getEntityLocation().getY());
        toReturn.getLocation().setZ(entityStatePdu.getEntityLocation().getZ());
        // entity orientation
        toReturn.setEntityOrientation(new Orientation());
        toReturn.getEntityOrientation().setPhi(entityStatePdu.getEntityOrientation().getPhi());
        toReturn.getEntityOrientation().setTheta(entityStatePdu.getEntityOrientation().getTheta());
        toReturn.getEntityOrientation().setPsi(entityStatePdu.getEntityOrientation().getPsi());
        // entity linear velocity
        toReturn.setLinearVelocity(new Vector3Float());
        toReturn.getLinearVelocity().setX(entityStatePdu.getEntityLinearVelocity().getX());
        toReturn.getLinearVelocity().setY(entityStatePdu.getEntityLinearVelocity().getY());
        toReturn.getLinearVelocity().setZ(entityStatePdu.getEntityLinearVelocity().getZ());
        // entity linear acceleration (from dead reckoning parameters)
        toReturn.setLinearAcceleration(new Vector3Float());
        toReturn.getLinearAcceleration().setX(entityStatePdu.getDeadReckoningParameters().getEntityLinearAcceleration().getX());
        toReturn.getLinearAcceleration().setY(entityStatePdu.getDeadReckoningParameters().getEntityLinearAcceleration().getY());
        toReturn.getLinearAcceleration().setZ(entityStatePdu.getDeadReckoningParameters().getEntityLinearAcceleration().getZ());
        // entity angular velocity (from dead reckoning parameters)
        toReturn.setAngularVelocity(new Vector3Float());
        toReturn.getAngularVelocity().setX(entityStatePdu.getDeadReckoningParameters().getEntityAngularVelocity().getX());
        toReturn.getAngularVelocity().setY(entityStatePdu.getDeadReckoningParameters().getEntityAngularVelocity().getY());
        toReturn.getAngularVelocity().setZ(entityStatePdu.getDeadReckoningParameters().getEntityAngularVelocity().getZ());
        return toReturn;
    }

    @Override
    protected void copyParametersToEntityState(DeadReckoningEntity source, EntityStatePdu target) {
        // timestamp
        target.setTimestamp(source.getTimestamp());
        // entity location
        target.getEntityLocation().setX(source.getLocation().getX());
        target.getEntityLocation().setY(source.getLocation().getY());
        target.getEntityLocation().setZ(source.getLocation().getZ());
        // entity orientation
        target.getEntityOrientation().setPhi(source.getEntityOrientation().getPhi());
        target.getEntityOrientation().setTheta(source.getEntityOrientation().getTheta());
        target.getEntityOrientation().setPsi(source.getEntityOrientation().getPsi());
        // entity linear velocity
        target.getEntityLinearVelocity().setX(source.getLinearVelocity().getX());
        target.getEntityLinearVelocity().setY(source.getLinearVelocity().getY());
        target.getEntityLinearVelocity().setZ(source.getLinearVelocity().getZ());
        // entity linear acceleration (from dead reckoning parameters)
        target.getDeadReckoningParameters().getEntityLinearAcceleration().setX(source.getLinearAcceleration().getX());
        target.getDeadReckoningParameters().getEntityLinearAcceleration().setY(source.getLinearAcceleration().getY());
        target.getDeadReckoningParameters().getEntityLinearAcceleration().setZ(source.getLinearAcceleration().getZ());
    }

    @Override
    protected void processArticulatedParameters(EntityStatePdu entityStatePdu, double deltaTime) {
        // Process articulation parameters
        for (ArticulationParameter thisRate : entityStatePdu.getArticulationParameters()) { // update articulation parameter values
            if (thisRate.getParameterType() % 2 == 0) {  // found a rate (even type)
                // Search for the corresponding art. part value parameter
                for (ArticulationParameter thisVal : entityStatePdu.getArticulationParameters()) {
                    if (thisVal.getParameterType() == thisRate.getParameterType() - 1 &&
                            thisVal.getPartAttachedTo() == thisRate.getPartAttachedTo()) {  // found part's value
                        thisVal.setParameterValue(
                                getParameterValue(
                                        deltaTime,
                                        thisVal.getParameterType() % 32,
                                        thisVal.getParameterValue(),
                                        thisVal.getParameterValueFirstSubfield(),
                                        thisRate.getParameterValue()
                                )
                        );
                        break;  // exit the value search loop
                    }  // end of found the corresponding value
                }  // end value search
            }  // end of found a rate 
        }  // end update articulation parameter values
    }
}
