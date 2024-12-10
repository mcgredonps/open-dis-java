package edu.nps.moves.deadreckoning;

import edu.nps.moves.dis7.Orientation;
import edu.nps.moves.dis7.Vector3Double;
import edu.nps.moves.dis7.Vector3Float;
import edu.nps.moves.disenum.DeadReckoningAlgorithm;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * AbstractDeadReckoner
 * <br>
 * Provides the common Dead Reckoning algorithm structure
 * @author Alessio Matricardi
 */
public abstract class AbstractDeadReckoner<T> implements IDeadReckoner<T> {

    private static final double MIN_ROTATION_RATE = 0.2 * Math.PI / 180;  // minimum significant rate = 1deg/5sec
    private static final double MIN_ACCELERATION_RATE = 0.1;  // minimum significant rate = 1m/l0sec^2

    @Override
    public void performDeadReckoning(T entityStatePdu, double deltaTime) {
        DeadReckoningEntity deadReckoningEntity = convertToDeadReckoningEntity(entityStatePdu);
        // common Dead Reckoning algorithm
        performDeadReckoning(deadReckoningEntity, deltaTime);
        // copy parameters from our custom entity to the Entity State PDU
        copyParametersToEntityState(deadReckoningEntity, entityStatePdu);
        // process articulated parameters algorithm phase
        processArticulatedParameters(entityStatePdu, deltaTime);
    }

    private void performDeadReckoning(DeadReckoningEntity deadReckoningEntity, double deltaTime) {
        if (!DeadReckoningAlgorithm.enumerationForValueExists(deadReckoningEntity.getDeadReckoningAlgorithm()) || deltaTime < 0.0) {
            throw new IllegalArgumentException();
        }

        // Determine the DR algorithm type to be used for this entity
        DeadReckoningAlgorithm deadReckoningAlgorithm = DeadReckoningAlgorithm.lookup[deadReckoningEntity.getDeadReckoningAlgorithm()];
        if (deadReckoningAlgorithm == DeadReckoningAlgorithm.OTHER) {
            throw new IllegalArgumentException();
        }

        if (deltaTime == 0.0) {
            // no time elapsed, nothing to do
            return;
        }

        // Update the ESPDU timestamp
        long deltaTimeStamp = 2 * (long) (deltaTime * 2147483648. / 3600.);
        deadReckoningEntity.setTimestamp(deadReckoningEntity.getTimestamp() + deltaTimeStamp);

        if (deadReckoningAlgorithm == DeadReckoningAlgorithm.STATIC_ENTITY_DOES_NOT_MOVE) {
            // no time elapsed, nothing further to do
            return;
        }

        // Change structure to vectors because they are easier to work with
        Vector3D location = new Vector3D(
                deadReckoningEntity.getLocation().getX(),
                deadReckoningEntity.getLocation().getY(),
                deadReckoningEntity.getLocation().getZ()
        );

        Vector3D velocity = new Vector3D(
                deadReckoningEntity.getLinearVelocity().getX(),
                deadReckoningEntity.getLinearVelocity().getY(),
                deadReckoningEntity.getLinearVelocity().getZ()
        );

        Vector3D acceleration = new Vector3D(
                deadReckoningEntity.getLinearAcceleration().getX(),
                deadReckoningEntity.getLinearAcceleration().getY(),
                deadReckoningEntity.getLinearAcceleration().getZ()
        );

        // Convert rotation rates to double
        double rotRateRoll = deadReckoningEntity.getAngularVelocity().getX();
        double rotRatePitch = deadReckoningEntity.getAngularVelocity().getY();
        double rotRateYaw = deadReckoningEntity.getAngularVelocity().getZ();

        // Check for rotation
        boolean rotating = false;
        if (deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMR_P_W ||
                deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMR_V_W ||
                deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMR_P_B ||
                deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMR_V_B) {
            if (Math.abs(rotRateRoll) >= MIN_ROTATION_RATE ||
                    Math.abs(rotRatePitch) >= MIN_ROTATION_RATE ||
                    Math.abs(rotRateYaw) >= MIN_ROTATION_RATE) {
                rotating = true;
            }
        }

        // Check for acceleration
        boolean accelerating = false;
        if (deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMR_V_W ||
                deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMF_V_W ||
                deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMR_V_B ||
                deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMF_V_B) {
            if (Math.abs(acceleration.getX()) >= MIN_ACCELERATION_RATE ||
                    Math.abs(acceleration.getY()) >= MIN_ACCELERATION_RATE ||
                    Math.abs(acceleration.getZ()) >= MIN_ACCELERATION_RATE) {
                accelerating = true;
            }
        }

        // Check for use of body (entity relative) vel/accel coords
        boolean bodyCoords = deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMF_P_B ||
                deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMR_P_B ||
                deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMR_V_B ||
                deadReckoningAlgorithm == DeadReckoningAlgorithm.DRMF_V_B;

        // Set up and compute intermediate variables
        Rotation WCStoECS1Mat = Rotation.IDENTITY;

        if (rotating || bodyCoords) {
            WCStoECS1Mat = new Rotation(
                    RotationOrder.ZYX,
                    RotationConvention.FRAME_TRANSFORM,
                    deadReckoningEntity.getEntityOrientation().getPsi(),
                    deadReckoningEntity.getEntityOrientation().getTheta(),
                    deadReckoningEntity.getEntityOrientation().getPhi()
            );
        }

        Vector3D vDistECS1 = Vector3D.ZERO;
        Vector3D aDistECS1 = Vector3D.ZERO;

        if (rotating) {

            // Compute initial ECS to final ECS rotation matrix
            RealVector wVec = MatrixUtils.createRealVector(new double[]{rotRateRoll, rotRatePitch, rotRateYaw});

            RealMatrix wOut = wVec.outerProduct(wVec);
            RealMatrix wMat = MatrixUtils.createRealMatrix(new double[][]{
                            {0.0, -rotRateYaw, rotRatePitch},
                            {rotRateYaw, 0.0, -rotRateRoll},
                            {-rotRatePitch, rotRateRoll, 0.0}
                    }
            );

            double wMagSq = wVec.dotProduct(wVec);
            double wMag = Math.sqrt(wMagSq);
            double wMagT = wMag * deltaTime;
            double cosWMagT = Math.cos(wMagT);
            double term1 = (1.0 - cosWMagT) / wMagSq;
            double sinWMagT = Math.sin(wMagT);
            double term3 = sinWMagT / wMag;

            RealMatrix ECS1toECS2Mat = wOut.scalarMultiply(term1)
                    .add(MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(cosWMagT))
                    .subtract(wMat.scalarMultiply(term3));
            if (bodyCoords) {
                // Compute the first integral of the final ECS to initial
                //   ECS rotation matrix for use with velocity
                double wMag3 = wMagSq * wMag;
                double vIntTerm1 = (wMagT - sinWMagT) / wMag3;
                // Terms 2&3 of this matrix calc are the same as terms 3&1 of
                // earlier initial to final ECS matrix calculation so we'll reuse
                RealMatrix vIntECS2toECS1Mat = wOut.scalarMultiply(vIntTerm1)
                        .add(MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(term3))
                        .add(wMat.scalarMultiply(term1));

                // Compute movement in initial entity coordinates due
                // to initial velocity (ignoring acceleration)
                // vDistECS1 = vIntECS2toECS1Mat * velocity, a little hard to read due to type changes...
                vDistECS1 = new Vector3D(vIntECS2toECS1Mat.operate(
                        MatrixUtils.createRealVector(velocity.toArray())).toArray());

                if (accelerating) {
                    // Compute first integral of the final ECS to initial ECS
                    //   rotation matrix times time for use with acceleration
                    double wMag4 = wMag3 * wMag;
                    double aIntTerm2Top = cosWMagT + (wMagT * sinWMagT) - 1.0;
                    double aIntTerm1 = ((0.5 * wMagT * wMagT) - aIntTerm2Top) / wMag4;
                    double aIntTerm2 = aIntTerm2Top / wMagSq;
                    double aIntTerm3 = (sinWMagT - (wMagT * cosWMagT)) / wMag3;

                    RealMatrix aIntECS2toECS1Mat = wOut.scalarMultiply(aIntTerm1)
                            .add(MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(aIntTerm2))
                            .add(wMat.scalarMultiply(aIntTerm3));

                    // Compute movement in initial entity coordinates due
                    //    to acceleration (ignoring initial velocity)
                    // aDistECS1 = aIntECS2toECS1Mat * acceleration, a little hard to read due to type changes...
                    aDistECS1 = new Vector3D(aIntECS2toECS1Mat.operate(
                            MatrixUtils.createRealVector(acceleration.toArray())).toArray());
                }
            }

            // Compute final Euler angles

            Rotation WCStoECS2Mat = new Rotation(ECS1toECS2Mat.getData(), 1e-15)
                    .applyTo(WCStoECS1Mat);

            double[] eulerAngles = WCStoECS2Mat.getAngles(RotationOrder.ZYX, RotationConvention.FRAME_TRANSFORM);
            // Update ESPDU Euler angle values
            deadReckoningEntity.getEntityOrientation().setPsi((float) eulerAngles[0]);
            deadReckoningEntity.getEntityOrientation().setTheta((float) eulerAngles[1]);
            deadReckoningEntity.getEntityOrientation().setPhi((float) eulerAngles[2]);

            // end of compute final Euler angles
        }  // if (rotating)

        // Compute final velocity and position
        if (bodyCoords) {  // velocity/acceleration in ECS coords
            if (!rotating) {
                // Compute movement in initial entity coordinates due
                //    to initial velocity (ignoring acceleration)
                vDistECS1 = velocity.scalarMultiply(deltaTime);
            }
            if (accelerating) {
                // Update ESPDU velocity values in final ECS coords
                velocity = velocity.add(deltaTime, acceleration);

                if (!rotating) {
                    // Compute movement in initial entity coordinates due
                    //    to acceleration (ignoring initial velocity)
                    aDistECS1 = acceleration.scalarMultiply(0.5 * deltaTime * deltaTime);
                }
            }  // end if accelerating block

            // Compute the total movement in initial ECS coords
            Vector3D distECS1 = new Vector3D(vDistECS1.add(aDistECS1).toArray());

            // Convert the movement in initial ECS coords to update ESPDU
            //    location in WCS coords.
            // Use transposed WCStoECSIMat rather than building an ECSItoWCSMat
            location = location.add(WCStoECS1Mat.applyInverseTo(distECS1));
        } else {  // velocity/acceleration in WCS coords
            if (accelerating) {
                // Compute final velocity vector in WCS coords
                Vector3D finalVelWCS = velocity.add(deltaTime, acceleration);

                // Update ESPDU location in WCS coords
                location = location.add(0.5 * deltaTime, velocity.add(finalVelWCS));

                // Update ESPDU velocity values
                velocity = finalVelWCS;
            } else // not speeding up, use first-order linear projection
            {
                location = location.add(deltaTime, velocity);
            }
        }

        // Store location, velocity, and acceleration back in ESPDU object
        deadReckoningEntity.getLocation().setX(location.getX());
        deadReckoningEntity.getLocation().setY(location.getY());
        deadReckoningEntity.getLocation().setZ(location.getZ());

        deadReckoningEntity.getLinearVelocity().setX((float) velocity.getX());
        deadReckoningEntity.getLinearVelocity().setY((float) velocity.getY());
        deadReckoningEntity.getLinearVelocity().setZ((float) velocity.getZ());

        deadReckoningEntity.getLinearAcceleration().setX((float) acceleration.getX());
        deadReckoningEntity.getLinearAcceleration().setY((float) acceleration.getY());
        deadReckoningEntity.getLinearAcceleration().setZ((float) acceleration.getZ());
    }

    protected abstract DeadReckoningEntity convertToDeadReckoningEntity(T entityStatePdu);

    protected abstract void copyParametersToEntityState(DeadReckoningEntity source, T target);

    protected abstract void processArticulatedParameters(T entityStatePdu, double deltaTime);

    protected static class DeadReckoningEntity {
        long timestamp;
        short deadReckoningAlgorithm;
        protected Vector3Double location;
        protected Orientation entityOrientation;
        protected Vector3Float linearVelocity;
        protected Vector3Float linearAcceleration;
        protected Vector3Float angularVelocity;

        public long getTimestamp() {
            return timestamp;
        }

        public void setTimestamp(long timestamp) {
            this.timestamp = timestamp;
        }

        public short getDeadReckoningAlgorithm() {
            return deadReckoningAlgorithm;
        }

        public void setDeadReckoningAlgorithm(short deadReckoningAlgorithm) {
            this.deadReckoningAlgorithm = deadReckoningAlgorithm;
        }

        public Vector3Double getLocation() {
            return location;
        }

        public void setLocation(Vector3Double location) {
            this.location = location;
        }

        public Orientation getEntityOrientation() {
            return entityOrientation;
        }

        public void setEntityOrientation(Orientation entityOrientation) {
            this.entityOrientation = entityOrientation;
        }

        public Vector3Float getLinearVelocity() {
            return linearVelocity;
        }

        public void setLinearVelocity(Vector3Float linearVelocity) {
            this.linearVelocity = linearVelocity;
        }

        public Vector3Float getLinearAcceleration() {
            return linearAcceleration;
        }

        public void setLinearAcceleration(Vector3Float linearAcceleration) {
            this.linearAcceleration = linearAcceleration;
        }

        public Vector3Float getAngularVelocity() {
            return angularVelocity;
        }

        public void setAngularVelocity(Vector3Float angularVelocity) {
            this.angularVelocity = angularVelocity;
        }
    }

    protected static double getParameterValue(double deltaTime, int parameterType, double oldValue, double oldValueFirstSubField, double rate) {
        double newValue;
        // update values based on the type
        switch (parameterType) {
            case 1:   // position
            case 3:  // extension
                newValue = oldValue + rate * deltaTime;
                if (newValue > 1.0) { // fully extended
                    newValue = 1.0;
                } else if (newValue < 0.0) { // fully retracted
                    newValue = 0.0;
                }
                break;
            case 5:  // x
            case 7:  // y
            case 9:  // z
                newValue = oldValue + rate * deltaTime;
                break;
            case 11:  // azimuth
            case 13:  // elevation
                newValue = oldValue + rate * deltaTime;
                if (oldValueFirstSubField > Math.PI) {  // wrapping over 180 degrees
                    newValue = newValue - 2.0 * Math.PI;
                } else if (oldValueFirstSubField < -Math.PI) {  // wrapping under 180 degrees
                    newValue = newValue + 2.0 * Math.PI;
                }
                break;
            case 15:  // rotation
                newValue = oldValue + rate * deltaTime;
                if (newValue > Math.PI) {  // wrapping over 180 degrees
                    newValue = newValue - 2.0 * Math.PI;
                } else if (newValue < -Math.PI) {  // wrapping under 180 degrees
                    newValue = newValue + 2.0 * Math.PI;
                }
                break;
            default:
                throw new IllegalArgumentException();
        }
        return newValue;
    }
}
