package edu.nps.moves.deadreckoning;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

/**
 * The root super class for all DIS Dead-Reckoning algorithms.
 * Based on the algorithms from the 
 * IEEE 1278_1-1995_DIS standards found in Annex B.
 * <p>
 * Creates an abstract instance of a Dead Reckoning (DR) algorithm, defined 
 * by the concrete Dead Reckoning algorithm on the right hand side.
 * <P>
 * At each PDU update received, call the set function to update the DR 
 * algorithm with the most accurate and updated information. Expected to receive 
 * a new update approximately every 5 seconds or so. Each PDU is essentially a 
 * restart or reset of the DR state.
 * <p>
 * The DR works off the last good state (origin) and extrapolates out from that 
 * point based on the velocity and acceleration parameters from the set
 * function.
 * <P>
 * The DR algorithm updates 30 times a second. The instantiating entity
 * can get updated DR states at its leisure up to 30 times a second by calling 
 * the get function, which returns an array of 6 doubles 3 x location and 
 * 3 x orientation. With these 6 parameters the entity can redraw itself in an
 * updated location and orientation based on its projected path.

 * <u>An Example:</u><br>
 * <pre>
import DIS.DeadReckoning.*;

public class runTest 
{
    public static void main(String s[])
    {
        // create a DeadReconing Entity
        DIS_DeadReckoning dr = new DIS_DR_FPW_02();

        // make the arrays of location and other parameters
        //                loc      orien    lin V    Accel    Ang V
        double[] locOr = {2,3,4,   5,6,1,   1,2,1,   0,0,0,   0,0,0};

        // set the parameters
        dr.setNewAll(locOr);

        // Print out the current state
        System.out.println(dr.toString());
        System.out.println();

        try
        {
            // wait 1 second
            Thread.sleep(1000);

            // request an update from the DR algorithm
            // should be original + 1 full value of other parameters
            // new position should be (3, 5, 5)
            double[] update = dr.getUpdatedPositionOrientation();

            // print the update to the screen
            System.out.println(dr.toString());        
        }
        catch(Exception e)
        {
            System.out.println("Unknow Error...?\n    " + e);
        }

        // terminate with exit to get out of the inf while loop
        System.exit(0);
    }
}

Resulting Output:
Current State of this Entity:
    Entity Location = (2.0, 3.0, 4.0)
    Entity Orientation = (5.0, 6.0, 1.0)
    Entity Linear Velocity = (1.0, 2.0, 1.0)
    Entity Linear Acceleration = (0.0, 0.0, 0.0)
    Entity Angular Velocity = (0.0, 0.0, 0.0)
    Delta between updates = 0.033333335

Current State of this Entity:
    Entity Location = (3.000000052154064, 5.000000104308128, 5.000000052154064)
    Entity Orientation = (5.0, 6.0, 1.0)
    Entity Linear Velocity = (1.0, 2.0, 1.0)
    Entity Linear Acceleration = (0.0, 0.0, 0.0)
    Entity Angular Velocity = (0.0, 0.0, 0.0)
    Delta between updates = 0.033333335


</pre>
 * 
 * @author Sheldon L. Snyder
 * @deprecated Use {@link DeadReckoner} instead.
 */
@Deprecated
public abstract class DIS_DeadReckoning implements Runnable
{
    /**
     * The entity's X coordinate location with double precision 64bit 
     */
    protected double entityLocation_X;
    /**
     * The entity's Y coordinate location with double precision 64bit 
     */
    protected double entityLocation_Y;
    /**
     * The entity's Z coordinate location with double precision 64bit 
     */    
    protected double entityLocation_Z;    

    /**
     * The X orientation of the entity with 32bit float
     */ 
    protected float entityOrientation_psi;
    /**
     * The Y orientation of the entity with 32bit float
     */ 
    protected float entityOrientation_theta;
    /**
     * The Z orientation of the entity with 32bit float
     */     
    protected float entityOrientation_phi;        

    /**
     * The X linear velocity 32bit float
     */ 
    protected float entityLinearVelocity_X = 0;
    /**
     * The Y linear velocity 32bit float
     */     
    protected float entityLinearVelocity_Y = 0;
    /**
     * The Z linear velocity 32bit float
     */     
    protected float entityLinearVelocity_Z = 0;   

    /**
     * The linear X acceleration 32bit float
     */ 
    protected float entityLinearAcceleration_X = 0;
    /**
     * The linear Y acceleration 32bit float
     */ 
    protected float entityLinearAcceleration_Y = 0;
    /**
     * The linear Z acceleration 32bit float
     */     
    protected float entityLinearAcceleration_Z = 0;  

    /**
     * The X angular velocity 32bit float
     */ 
    protected float entityAngularVelocity_X = 0;
    /**
     * The Y angular velocity 32bit float
     */ 
    protected float entityAngularVelocity_Y = 0;
    /**
     * The Z angular velocity 32bit float
     */     
    protected float entityAngularVelocity_Z = 0;   

    /**
     * how may times per second to update this entity's position
     */
    protected float fps = 30;

    /**
     * How far to change the location/orientation per update
     */
    protected float changeDelta = 1f/fps;
    /**
     * How many updates have occured ... only used for testing
     * <p>
     * Reset to 0 with each call to setAll()
     */
    protected  int deltaCt = 0;// how many updates have been called
    /**
     * How long to wait between updates
     * <P>
     *  the delta between calls...how fast an entity will be updated
     * 
     * - Assumed a desired rate of 30 fps
     * - Given from the standard that all parameters are in meters/s
     * - To move 1 meter/second with 30 increments = 1/30 Delta between updates
     * - delay in milliseconds is 1/30 * 1000 || 1000 / 30
     * <p>
     * Note from Java Doc for JDK: <br>
     * Causes the currently executing thread to sleep (temporarily cease 
     * execution) for the specified number of milliseconds, subject to the 
     * precision and accuracy of system timers and schedulers. The thread does 
     * not lose ownership of any monitors.      
     */
    protected long stall = (long)(1000/fps);

    /**
     * Thread for the DR algorithm update timing (1/30 second)
     */
    protected Thread aThread;

    /**
     * the initial orientation, constant between delta T
     * Only changes when a setNewAll is called
     */
    Rotation initOrien;    

    /**
     * SKEW matrix, constant between delta T
     * Only changes when a setNewAll is called
     */
    RealMatrix skewOmega;

    /**
     * Angular velocity matrix, constant between delta T
     * Only changes when a setNewAll is called
     */
    RealMatrix ww;

    /**
     * Angular velocity magnitude, constant between delta T
     * Only changes when a setNewAll is called
     */
    double wMag;    
    /**
     * Magnitude of angular velocity squared
     */
    double wSq;

    /**
     * DIS timestamp
     */
    private long initTimestamp;
    
    static double MIN_ROTATION_RATE = 0.2 * Math.PI / 180;  // minimum significant rate = 1deg/5sec

    /***************************************************************************
     * Constructor for all DR algorithms...
     * <P>
     * Each subclass DR algorithm has a no arguments constructor, but all it 
     * does is call the super, i.e. this constructor, which establishes the
     * Thread
     */
    public DIS_DeadReckoning()
    {
        aThread = new Thread(this);
        aThread.start();
    }//DIS_DeadReckoning()------------------------------------------------------

    /***************************************************************************
     * Gets the revised position and orientation of this entity
     * <p>
     * Applies the required DR formula to the initial position and orientation 
     * of this entity and returns the updated location and position.
     * <p>
     * This function does not actually perform the computations, it only returns
     * the current state of the entity. The entity state is updated by the 
     * specified DR algorithm within the DR class behind the scenes. Updates are
     * created every 1/30 seconds.
     * 
     * NOTE: The concrete classes implementing the various DR algorithms are not
     * (currently) thread safe.  As a result, the getter functions may return a
     * result while the behind the scenes update is still in progress,
     * i.e. this getter functions may sample an inconsistent state in which
     * some fields have been updated to the current time sample while other
     * fields still pertain to the previous time sample.
     * 
     * Assume a desired rate of 30 fps
     * All parameters are in meters/s
     * to move 1 meter/second with 30 increments = 1/30 Delta between updates
     * 
     * Only returns an array of location and orientation because that
     * is all that is needed to update the location of the entity.  All other 
     * DR inputs are parameters for solving the location and orientation and so
     * are not returned, only set.
     *
     * Order of the returned array elements
     * 
     * - entityLocation_X
     * - entityLocation_Y
     * - entityLocation_Z
     * - entityOrientation_psi
     * - entityOrientation_theta
     * - entityOrientation_phi
     * 
     * @return -  6 doubles of location and orientation
     */
    public double[] getUpdatedPositionOrientation()
    {    
        double[] newLoc = {entityLocation_X, entityLocation_Y, entityLocation_Z,
                entityOrientation_psi, entityOrientation_theta, entityOrientation_phi};
        return newLoc;
    }//getUpdatedPositionOrientation()------------------------------------------

    /**
     * @return e doubles of velocity
     */
    public double[] getUpdatedVelocity()
    {    
        return new double[] { entityLinearVelocity_X, entityLinearVelocity_Y, entityLinearVelocity_Z };
    }//getUpdatedVelocity()------------------------------------------

    public long getUpdatedTimestamp() {
        long dtimeStamp = 2 * (long) (deltaCt * changeDelta * 2147483648. / 3600.);
        return initTimestamp + dtimeStamp;
    }
    
    /***************************************************************************
     * Sets the refresh rate for the scene.
     * <p>
     * Default is 30 but can be changed through this function call
     * 
     * @param frames - the number of updates per second to make
     */
    public void setFPS(int frames)
    {
        fps = frames;
        changeDelta = 1/fps;
    }//setFPS(int frames)-------------------------------------------------------



    /***************************************************************************
     * Set the parameters for this entity's DR function based on the most
     * recent PDU.
     * <p>
     * This is called by the entity any time the entity receives an updated
     * ESPDU for this entity.
     * <P>
     * This can be the first and initialization call or update. 
     * <P>
     * The following (triples) are set with each call:
     * 
     * - Entity Location 64bit
     * - Entity Orientation 32bit
     * - Entity Linear Velocity 32bit
     * - Entity Linear Acceleration 32bit
     * - Entity Angular Velocity 32bit

     * <P>
     * entityLocation_X = allDis[0];<br>
     * entityLocation_Y = allDis[1];<br>
     * entityLocation_Z = allDis[2];<br>
     * <p>
     * entityOrientation_psi = (float)allDis[3];<br>
     * entityOrientation_theta = (float)allDis[4];<br>
     * entityOrientation_phi = (float)allDis[5];<br>
     * <p>
     * entityLinearVelocity_X = (float)allDis[6];<br>
     * entityLinearVelocity_Y = (float)allDis[7];<br>
     * entityLinearVelocity_Z = (float)allDis[8];<br>
     * <p>
     * entityLinearAcceleration_X = (float)allDis[9];<br>
     * entityLinearAcceleration_Y = (float)allDis[10];<br>
     * entityLinearAcceleration_Z = (float)allDis[11];<br>
     * <p>
     * entityAngularVelocity_X = (float)allDis[12];<br>
     * entityAngularVelocity_Y = (float)allDis[13];<br>
     * entityAngularVelocity_Z = (float)allDis[14];<br>
     * <P>
     * DR fields from a PDU update or initial 
     * 
     * @param allDis - 15 double precisions representing the above in order of the above
     */
    public void setNewAll(double[] allDis)
    {
        entityLocation_X = allDis[0];
        entityLocation_Y = allDis[1];
        entityLocation_Z = allDis[2];

        entityOrientation_psi = (float)allDis[3];
        entityOrientation_theta = (float)allDis[4];
        entityOrientation_phi = (float)allDis[5];

        entityLinearVelocity_X = (float)allDis[6];
        entityLinearVelocity_Y = (float)allDis[7];
        entityLinearVelocity_Z = (float)allDis[8];

        entityLinearAcceleration_X = (float)allDis[9];
        entityLinearAcceleration_Y = (float)allDis[10];
        entityLinearAcceleration_Z = (float)allDis[11];

        entityAngularVelocity_X = (float)allDis[12];
        entityAngularVelocity_Y = (float)allDis[13];
        entityAngularVelocity_Z = (float)allDis[14];

        // solve for magnitude
        wSq = entityAngularVelocity_X * entityAngularVelocity_X +
                entityAngularVelocity_Y * entityAngularVelocity_Y +
                entityAngularVelocity_Z * entityAngularVelocity_Z;
        wMag = Math.sqrt(wSq);

        //System.out.println("wMag print");
        //System.out.println(wMag);
        //System.out.println();

        // build the skew matrix
        setOmega();
        //System.out.println("skewOmega print");
        //skewOmega.print();
        //System.out.println();

        // build the angular velocity matrix
        setWW();
        //System.out.println("ww print");
        //ww.print();
        //System.out.println();

        // reset delta count given this new update        
        setInitOrient();
        //System.out.println("init Orient print");
        //initOrien.print();
        //System.out.println();        

        deltaCt = 0;
    }//setNewAll(double[] allDis)-----------------------------------------------

    /**
     * Set the timestamp
     * @param initTimestamp
     */
    public void setInitTimestamp(long initTimestamp) {
        this.initTimestamp = initTimestamp;
    }

    /***************************************************************************
     * With each setNewAll() makes the new initial orientation matrix given the
     * new parameters
     */    
    private void setInitOrient()
    {
        initOrien = new Rotation(
                RotationOrder.ZYX,
                RotationConvention.FRAME_TRANSFORM,
                entityOrientation_psi,
                entityOrientation_theta,
                entityOrientation_phi);
    }



    /***************************************************************************
     * With each setNewAll() makes the new angular velocity matrix given the
     * new parameters
     */
    private void setWW()
    {
        ww = MatrixUtils.createRealMatrix(3, 3);
        ww.setEntry(0, 0, entityAngularVelocity_X * entityAngularVelocity_X);
        ww.setEntry(0, 1, entityAngularVelocity_X * entityAngularVelocity_Y);
        ww.setEntry(0, 2, entityAngularVelocity_X * entityAngularVelocity_Z);                
        ww.setEntry(1, 0, entityAngularVelocity_Y * entityAngularVelocity_X);
        ww.setEntry(1, 1, entityAngularVelocity_Y * entityAngularVelocity_Y);
        ww.setEntry(1, 2, entityAngularVelocity_Y * entityAngularVelocity_Z);
        ww.setEntry(2, 0, entityAngularVelocity_Z * entityAngularVelocity_X);
        ww.setEntry(2, 1, entityAngularVelocity_Z * entityAngularVelocity_Y);
        ww.setEntry(2, 2, entityAngularVelocity_Z * entityAngularVelocity_Z);        
    }



    /***************************************************************************
     * With each setNewAll() makes the new skew matrix given the
     * new parameters
     */
    private void setOmega()
    {
        skewOmega = MatrixUtils.createRealMatrix(3, 3);
        skewOmega.setEntry(0, 0, 0);
        skewOmega.setEntry(1, 1, 0);
        skewOmega.setEntry(2, 2, 0);        
        skewOmega.setEntry(1, 0, entityAngularVelocity_Z);
        skewOmega.setEntry(2, 0, -entityAngularVelocity_Y);
        skewOmega.setEntry(2, 1, entityAngularVelocity_X);              
        skewOmega.setEntry(0, 1, -entityAngularVelocity_Z);
        skewOmega.setEntry(0, 2, entityAngularVelocity_Y);
        skewOmega.setEntry(1, 2, -entityAngularVelocity_X);          
    }



    /***************************************************************************
     * Pretty print the current state of this Dead Reckoning object
     * <p>
     * Updates are not included in this call, this is only the state. 
     * 
     * @return - String of pretty print of this DR entity
     */
    public String toString()
    {
        String buff = "Current State of this Entity:\n" +
                "    Entity Location = (" + entityLocation_X + ", " +
                entityLocation_Y + ", " + entityLocation_Z + ")\n" +

            "    Entity Orientation = (" + entityOrientation_psi + ", " +
            entityOrientation_theta + ", " + entityOrientation_phi + ")\n" +

            "    Entity Linear Velocity = (" + entityLinearVelocity_X + ", " +
            entityLinearVelocity_Y + ", " + entityLinearVelocity_Z + ")\n" +

            "    Entity Linear Acceleration = (" + entityLinearAcceleration_X + ", " +
            entityLinearAcceleration_Y + ", " + entityLinearAcceleration_Z + ")\n" +

            "    Entity Angular Velocity = (" + entityAngularVelocity_X + ", " + 
            entityAngularVelocity_Y + ", " + entityAngularVelocity_Z + ")\n" +

            "    Delta between updates = " + changeDelta;

        return buff;
    }// toString()--------------------------------------------------------------
}// DIS_DeadReckoning-----------------------------------------------------------
