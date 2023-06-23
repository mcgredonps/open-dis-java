package edu.nps.moves.dis;

import java.io.*;

/**
 * Section 5.2.5. Articulation parameters for movable parts and attached parts
 * of an entity. Specifes wether or not a change has occured, the part
 * identifcation of the articulated part to which it is attached, and the type
 * and value of each parameter.
 *
 * Copyright (c) 2008-2016, MOVES Institute, Naval Postgraduate School. All
 * rights reserved. This work is licensed under the BSD open source license,
 * available at https://www.movesinstitute.org/licenses/bsd.html
 *
 * @author DMcG
 */
public class ArticulationParameter extends Object implements Serializable {

    protected short parameterTypeDesignator;

    protected short changeIndicator;

    protected int partAttachedTo;

    protected int parameterType;

    protected double parameterValue;

    protected EntityType entityType;

    protected final int ARTICULATED_PART = 0;

    protected final int ATTACHED_PART = 1;

    /**
     * Constructor
     */
    public ArticulationParameter() {
    }

    public int getMarshalledSize() {
        int marshalSize = 0;

        marshalSize = marshalSize + 1;  // parameterTypeDesignator
        marshalSize = marshalSize + 1;  // changeIndicator
        marshalSize = marshalSize + 2;  // partAttachedTo
        marshalSize = marshalSize + 4;  // parameterType
        marshalSize = marshalSize + 8;  // parameterValue

        return marshalSize;
    }

    public void setParameterTypeDesignator(short pParameterTypeDesignator) {
        parameterTypeDesignator = pParameterTypeDesignator;
    }

    public short getParameterTypeDesignator() {
        return parameterTypeDesignator;
    }

    public void setChangeIndicator(short pChangeIndicator) {
        changeIndicator = pChangeIndicator;
    }

    public short getChangeIndicator() {
        return changeIndicator;
    }

    public void setPartAttachedTo(int pPartAttachedTo) {
        partAttachedTo = pPartAttachedTo;
    }

    public int getPartAttachedTo() {
        return partAttachedTo;
    }

    public void setParameterType(int pParameterType) {
        parameterType = pParameterType;
    }

    public int getParameterType() {
        return parameterType;
    }

    public void setParameterValue(double pParameterValue) {
        parameterValue = pParameterValue;
    }

    public double getParameterValue() {
        return parameterValue;
    }

    public EntityType getEntityType() {
        return entityType;
    }

    public void setEntityType(EntityType entityType) {
        this.entityType = entityType;
    }

// From the spec DIS IEEE Std 1278.1-199
//
// A.2.1.8 Parameter Value field
// The 64-bit Parameter Value field is divided into two 32-bit subfields. The most-significant 32-bit subfield
// represents a 32-bit floating point number. The interpretation of this subfield depends on the value of type
// metric as specified in A.2.1.4. The least significant 32-bit subfield shall be zero.
    public float getParameterValueFirstSubfield() {
        return (float) getParameterValue();
    }

// From the spec DIS IEEE Std 1278.1-199
//
//An articulated parameter type consists of two components (figure A.1). The first component, consisting of
//the least significant 5 bits of the Parameter Type field, defines the type metric. The type metric determines
//which of the transformations described in A.2.1.4 are specified by this parameter type. The second component,
//consisting of the next 27 bits of the Parameter Type field, defines the type class.
    public int getParameterTypeMetric() {
        return getParameterType() & 0x1f;
    }

    public int getParameterTypeClass() {
        return getParameterType() >>> 5;
    }

    public int getArticulatedPartIndex() {
        return getParameterType() - getParameterTypeMetric();
    }

    /**
     * Packs a Pdu into the ByteBuffer.
     *
     * @throws java.nio.BufferOverflowException if buff is too small
     * @throws java.nio.ReadOnlyBufferException if buff is read only
     * @see java.nio.ByteBuffer
     * @param buff The ByteBuffer at the position to begin writing
     * @since ??
     */
    public void marshal(java.nio.ByteBuffer buff) {
        buff.put((byte) parameterTypeDesignator);
        buff.put((byte) changeIndicator);
        buff.putShort((short) partAttachedTo);
        buff.putInt((int) parameterType);
        if (parameterTypeDesignator == ARTICULATED_PART) {
            buff.putFloat((float) parameterValue);
            buff.putFloat((float) 0);
        } else if (parameterTypeDesignator == ATTACHED_PART) {
            entityType.marshal(buff);
        }

    } // end of marshal method

    /**
     * Unpacks a Pdu from the underlying data.
     *
     * @throws java.nio.BufferUnderflowException if buff is too small
     * @see java.nio.ByteBuffer
     * @param buff The ByteBuffer at the position to begin reading
     * @since ??
     */
    public void unmarshal(java.nio.ByteBuffer buff) {
        parameterTypeDesignator = (short) (buff.get() & 0xFF);
        changeIndicator = (short) (buff.get() & 0xFF);
        partAttachedTo = (int) (buff.getShort() & 0xFFFF);
        parameterType = buff.getInt();
        if (parameterTypeDesignator == ARTICULATED_PART) {
            parameterValue = buff.getFloat();
            buff.getFloat();
        } else if (parameterTypeDesignator == ATTACHED_PART) {
            entityType.unmarshal(buff);
        }
    } // end of unmarshal method 


    /*
  * The equals method doesn't always work--mostly it works only on classes that consist only of primitives. Be careful.
     */
    @Override
    public boolean equals(Object obj) {

        if (this == obj) {
            return true;
        }

        if (obj == null) {
            return false;
        }

        if (getClass() != obj.getClass()) {
            return false;
        }

        return equalsImpl(obj);
    }

    /**
     * Compare all fields that contribute to the state, ignoring transient and
     * static fields, for <code>this</code> and the supplied object
     *
     * @param obj the object to compare to
     * @return true if the objects are equal, false otherwise.
     */
    public boolean equalsImpl(Object obj) {
        boolean ivarsEqual = true;

        if (!(obj instanceof ArticulationParameter)) {
            return false;
        }

        final ArticulationParameter rhs = (ArticulationParameter) obj;

        if (!(parameterTypeDesignator == rhs.parameterTypeDesignator)) {
            ivarsEqual = false;
        }
        if (!(changeIndicator == rhs.changeIndicator)) {
            ivarsEqual = false;
        }
        if (!(partAttachedTo == rhs.partAttachedTo)) {
            ivarsEqual = false;
        }
        if (!(parameterType == rhs.parameterType)) {
            ivarsEqual = false;
        }
        if (!(parameterValue == rhs.parameterValue)) {
            ivarsEqual = false;
        }

        if (!(entityType.equalsImpl(rhs.getEntityType()))) {
            ivarsEqual = false;
        }

        return ivarsEqual;
    }
} // end of class
