package edu.nps.moves.dis;

import java.util.*;
import java.io.*;
import java.nio.ByteBuffer;

/**
 * Section 5.3.8.1. Detailed information about a radio transmitter. This PDU
 * requires manually written code to complete, since the modulation parameters
 * are of variable length. UNFINISHED
 *
 * Copyright (c) 2008-2016, MOVES Institute, Naval Postgraduate School. All
 * rights reserved. This work is licensed under the BSD open source license,
 * available at https://www.movesinstitute.org/licenses/bsd.html
 *
 * @author DMcG
 */
public class TransmitterPdu extends RadioCommunicationsFamilyPdu implements Serializable {

    /**
     * ID of the entity that is the source of the communication, ie contains the
     * radio
     */
    protected EntityID entityId = new EntityID();

    /**
     * particular radio within an entity
     */
    protected int radioId;

    /**
     * linear accelleration of entity
     */
    protected RadioEntityType radioEntityType = new RadioEntityType();

    /**
     * transmit state
     */
    protected short transmitState;

    /**
     * input source
     */
    protected short inputSource;

    /**
     * padding
     */
    protected int padding1;

    /**
     * Location of antenna
     */
    protected Vector3Double antennaLocation = new Vector3Double();

    /**
     * relative location of antenna, in entity coordinates
     */
    protected Vector3Float relativeAntennaLocation = new Vector3Float();

    /**
     * antenna pattern type
     */
    protected int antennaPatternType;

    /**
     * atenna pattern length
     */
    protected int antennaPatternCount;

    /**
     * frequency
     */
    protected long frequency;

    /**
     * transmit frequency Bandwidth
     */
    protected float transmitFrequencyBandwidth;

    /**
     * transmission power
     */
    protected float power;

    /**
     * modulation
     */
    protected ModulationType modulationType = new ModulationType();

    /**
     * crypto system enumeration
     */
    protected int cryptoSystem;

    /**
     * crypto system key identifer
     */
    protected int cryptoKeyId;

    /**
     * how many modulation parameters we have
     */
    protected short modulationParameterCount;

    /**
     * padding2
     */
    protected int padding2 = (int) 0;

    /**
     * padding3
     */
    protected short padding3 = (short) 0;

    /**
     * variable length list of modulation parameters
     */
    protected List< Short> modulationParametersList = new ArrayList< Short>();
    /**
     * variable length list of antenna pattern records
     */
    protected List< BeamAntennaPattern> antennaPatternList = new ArrayList< BeamAntennaPattern>();

    protected List<SphericalHarmonicAntennaPattern> sphericalHarmonicAntennaPatternList = new ArrayList<SphericalHarmonicAntennaPattern>();

    protected List<CcttSincgarsModulationParameters> cctSincarsModulationParametersList = new ArrayList<CcttSincgarsModulationParameters>();

    protected List<JtidsMidsModulationParameters> jtidsMidsModulationParametersList = new ArrayList<JtidsMidsModulationParameters>();

    /**
     * Constructor
     */
    public TransmitterPdu() {
        setPduType((short) 25);
    }

    public int getMarshalledSize() {
        int marshalSize = 0;
        antennaPatternCount = 0;
        marshalSize = super.getMarshalledSize();
        marshalSize = marshalSize + entityId.getMarshalledSize();  // entityId
        marshalSize = marshalSize + 2;  // radioId
        marshalSize = marshalSize + radioEntityType.getMarshalledSize();  // radioEntityType
        marshalSize = marshalSize + 1;  // transmitState
        marshalSize = marshalSize + 1;  // inputSource
        marshalSize = marshalSize + 2;  // padding1
        marshalSize = marshalSize + antennaLocation.getMarshalledSize();  // antennaLocation
        marshalSize = marshalSize + relativeAntennaLocation.getMarshalledSize();  // relativeAntennaLocation
        marshalSize = marshalSize + 2;  // antennaPatternType
        marshalSize = marshalSize + 2;  // antennaPatternCount
        marshalSize = marshalSize + 8;  // frequency
        marshalSize = marshalSize + 4;  // transmitFrequencyBandwidth
        marshalSize = marshalSize + 4;  // power
        marshalSize = marshalSize + modulationType.getMarshalledSize();  // modulationType
        marshalSize = marshalSize + 2;  // cryptoSystem
        marshalSize = marshalSize + 2;  // cryptoKeyId
        marshalSize = marshalSize + 1;  // modulationParameterCount
        marshalSize = marshalSize + 2;  // padding2
        marshalSize = marshalSize + 1;  // padding3

        int nrOfModulationBytes = 0;
        switch (getModulationType().getSystem()) {
            case 6:
                for (int idx = 0; idx < cctSincarsModulationParametersList.size(); idx++) {
                    CcttSincgarsModulationParameters listElement = cctSincarsModulationParametersList.get(idx);
                    marshalSize = marshalSize + listElement.getMarshalledSize();
                    nrOfModulationBytes = nrOfModulationBytes + listElement.getMarshalledSize();
                }
                break;
            case 8:
                for (int idx = 0; idx < jtidsMidsModulationParametersList.size(); idx++) {
                    JtidsMidsModulationParameters listElement = jtidsMidsModulationParametersList.get(idx);
                    marshalSize = marshalSize + listElement.getMarshalledSize();
                    nrOfModulationBytes = nrOfModulationBytes + listElement.getMarshalledSize();
                }
                break;
            default:
                marshalSize = marshalSize + (modulationParametersList.size() * 2);
                nrOfModulationBytes = nrOfModulationBytes + (modulationParametersList.size() * 2);
                break;
        }

        if (nrOfModulationBytes % 8 > 0) {
            int remainder = nrOfModulationBytes % 8;
            switch (remainder) {
                case 1:
                    marshalSize = marshalSize + 7;
                    break;
                case 2:
                    marshalSize = marshalSize + 6;
                    break;
                case 3:
                    marshalSize = marshalSize + 5;
                    break;
                case 4:
                    marshalSize = marshalSize + 4;
                    break;
                case 5:
                    marshalSize = marshalSize + 3;
                    break;
                case 6:
                    marshalSize = marshalSize + 2;
                    break;
                case 7:
                    marshalSize = marshalSize + 1;
                    break;
            }
        }
        switch (antennaPatternType) {
            case 0:
                //Omni-directional - no entries
                break;
            case 1:
                for (int idx = 0; idx < antennaPatternList.size(); idx++) {
                    BeamAntennaPattern listElement = antennaPatternList.get(idx);
                    marshalSize = marshalSize + listElement.getMarshalledSize();
                    antennaPatternCount += listElement.getMarshalledSize();
                }  // end of list marshalling
                if ((antennaPatternList.size() & 1) != 0) {//Checking for odd number of BeamAntennaPatterns and then add padding
                    marshalSize = marshalSize + 4;
                    antennaPatternCount += 4;
                }
                break;
            case 2:
                int nrOf16BitBlocks = 0;
                for (int idx = 0; idx < sphericalHarmonicAntennaPatternList.size(); idx++) {
                    SphericalHarmonicAntennaPattern listElement = sphericalHarmonicAntennaPatternList.get(idx);
                    marshalSize = marshalSize + listElement.getMarshalledSize();
                    antennaPatternCount += listElement.getMarshalledSize();
                    int n = listElement.getHarmonicOrder();
                    nrOf16BitBlocks++;
                    nrOf16BitBlocks = nrOf16BitBlocks + 2 * listElement.getCoefficientsList().size();
                }
                if (nrOf16BitBlocks % 4 == 3) {//Missing 1 16 bit block to end on 64 bit boundry
                    marshalSize = marshalSize + 2;
                    antennaPatternCount += 2;
                } else if (nrOf16BitBlocks % 4 == 1) {//missing 3 16 bit blocks to end on 64 bit boundry
                    marshalSize = marshalSize + 6;
                    antennaPatternCount += 6;
                } else if (nrOf16BitBlocks % 4 == 2) {//missing 2 16 bit blocks to end on 64 bit boundry
                    marshalSize = marshalSize + 4;
                    antennaPatternCount += 4;
                }
                break;
        }
        return marshalSize;
    }

    public void setEntityId(EntityID pEntityId) {
        entityId = pEntityId;
    }

    public EntityID getEntityId() {
        return entityId;
    }

    public void setRadioId(int pRadioId) {
        radioId = pRadioId;
    }

    public int getRadioId() {
        return radioId;
    }

    public void setRadioEntityType(RadioEntityType pRadioEntityType) {
        radioEntityType = pRadioEntityType;
    }

    public RadioEntityType getRadioEntityType() {
        return radioEntityType;
    }

    public void setTransmitState(short pTransmitState) {
        transmitState = pTransmitState;
    }

    public short getTransmitState() {
        return transmitState;
    }

    public void setInputSource(short pInputSource) {
        inputSource = pInputSource;
    }

    public short getInputSource() {
        return inputSource;
    }

    public void setPadding1(int pPadding1) {
        padding1 = pPadding1;
    }

    public int getPadding1() {
        return padding1;
    }

    public void setAntennaLocation(Vector3Double pAntennaLocation) {
        antennaLocation = pAntennaLocation;
    }

    public Vector3Double getAntennaLocation() {
        return antennaLocation;
    }

    public void setRelativeAntennaLocation(Vector3Float pRelativeAntennaLocation) {
        relativeAntennaLocation = pRelativeAntennaLocation;
    }

    public Vector3Float getRelativeAntennaLocation() {
        return relativeAntennaLocation;
    }

    public void setAntennaPatternType(int pAntennaPatternType) {
        antennaPatternType = pAntennaPatternType;
    }

    public int getAntennaPatternType() {
        return antennaPatternType;
    }

    public int getAntennaPatternCount() {
        return (int) antennaPatternList.size();
    }

    /**
     * Note that setting this value will not change the marshalled value. The
     * list whose length this describes is used for that purpose. The
     * getantennaPatternCount method will also be based on the actual list
     * length rather than this value. The method is simply here for java bean
     * completeness.
     */
    public void setAntennaPatternCount(int pAntennaPatternCount) {
        antennaPatternCount = pAntennaPatternCount;
    }

    public void setFrequency(long pFrequency) {
        frequency = pFrequency;
    }

    public long getFrequency() {
        return frequency;
    }

    public void setTransmitFrequencyBandwidth(float pTransmitFrequencyBandwidth) {
        transmitFrequencyBandwidth = pTransmitFrequencyBandwidth;
    }

    public float getTransmitFrequencyBandwidth() {
        return transmitFrequencyBandwidth;
    }

    public void setPower(float pPower) {
        power = pPower;
    }

    public float getPower() {
        return power;
    }

    public void setModulationType(ModulationType pModulationType) {
        modulationType = pModulationType;
    }

    public ModulationType getModulationType() {
        return modulationType;
    }

    public void setCryptoSystem(int pCryptoSystem) {
        cryptoSystem = pCryptoSystem;
    }

    public int getCryptoSystem() {
        return cryptoSystem;
    }

    public void setCryptoKeyId(int pCryptoKeyId) {
        cryptoKeyId = pCryptoKeyId;
    }

    public int getCryptoKeyId() {
        return cryptoKeyId;
    }

    public short getModulationParameterCount() {
        return modulationParameterCount;
    }

    /**
     * Note that setting this value will not change the marshalled value. The
     * list whose length this describes is used for that purpose. The
     * getmodulationParameterCount method will also be based on the actual list
     * length rather than this value. The method is simply here for java bean
     * completeness.
     */
    public void setModulationParameterCount(short pModulationParameterCount) {
        modulationParameterCount = pModulationParameterCount;
    }

    public void setPadding2(int pPadding2) {
        padding2 = pPadding2;
    }

    public int getPadding2() {
        return padding2;
    }

    public void setPadding3(short pPadding3) {
        padding3 = pPadding3;
    }

    public short getPadding3() {
        return padding3;
    }

    public void setModulationParametersList(List<Short> pModulationParametersList) {
        modulationParametersList = pModulationParametersList;
    }

    public List<Short> getModulationParametersList() {
        return modulationParametersList;
    }

    public void setAntennaPatternList(List<BeamAntennaPattern> pAntennaPatternList) {
        antennaPatternList = pAntennaPatternList;
    }

    public List<BeamAntennaPattern> getAntennaPatternList() {
        return antennaPatternList;
    }

    public void setSphericalHarmonicAntennaPatternList(List<SphericalHarmonicAntennaPattern> aSphericalHarmonicAntennaPatternList) {
        sphericalHarmonicAntennaPatternList = aSphericalHarmonicAntennaPatternList;
    }

    public List<SphericalHarmonicAntennaPattern> getSphericalHarmonicAntennaPatternList() {
        return sphericalHarmonicAntennaPatternList;
    }

    public List<CcttSincgarsModulationParameters> getCctSincarsModulationParametersList() {
        return cctSincarsModulationParametersList;
    }

    public void setCctSincarsModulationParametersList(List<CcttSincgarsModulationParameters> list) {
        cctSincarsModulationParametersList = list;
    }

    public List<JtidsMidsModulationParameters> getJtidsMidsModulationParametersList() {
        return jtidsMidsModulationParametersList;
    }

    public void setJtidsMidsModulationParameters(List<JtidsMidsModulationParameters> list) {
        jtidsMidsModulationParametersList = list;
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
        super.marshal(buff);
        entityId.marshal(buff);
        buff.putShort((short) radioId);
        radioEntityType.marshal(buff);
        buff.put((byte) transmitState);
        buff.put((byte) inputSource);
        buff.putShort((short) padding1);
        antennaLocation.marshal(buff);
        relativeAntennaLocation.marshal(buff);
        buff.putShort((short) antennaPatternType);
        buff.putShort((short) antennaPatternCount);
        buff.putLong((long) frequency);
        buff.putFloat((float) transmitFrequencyBandwidth);
        buff.putFloat((float) power);
        modulationType.marshal(buff);
        buff.putShort((short) cryptoSystem);
        buff.putShort((short) cryptoKeyId);
        buff.put((byte) (modulationParameterCount));
        buff.putShort((short) padding2);
        buff.put((byte) padding3);

        int nrOfModulationBytes = 0;
        switch (getModulationType().getSystem()) {
            case 6:                                  // CCTT SINCGARS
                for (int idx = 0; idx < cctSincarsModulationParametersList.size(); idx++) {
                    CcttSincgarsModulationParameters parameterRecord = cctSincarsModulationParametersList.get(idx);
                    parameterRecord.marshal(buff);
                    nrOfModulationBytes = nrOfModulationBytes + parameterRecord.getMarshalledSize();
                }
                break;
            case 8:                                  // JTIDS/MIDS
                for (int idx = 0; idx < jtidsMidsModulationParametersList.size(); idx++) {
                    JtidsMidsModulationParameters parameterRecord = jtidsMidsModulationParametersList.get(idx);
                    parameterRecord.marshal(buff);
                    nrOfModulationBytes = nrOfModulationBytes + parameterRecord.getMarshalledSize();
                }
                break;
            case 0:                                  // Other
            case 1:                                  // Generic
            case 2:                                  // HQ
            case 3:                                  // HQII
            case 4:                                  // HQIIA
            case 5:                                  // SINCGARS
            case 7:                                  // EPLRS
            default:
                for (int idx = 0; idx < modulationParametersList.size(); idx++) {
                    Short modulationParameter = modulationParametersList.get(idx);
                    buff.putShort(modulationParameter);
                    nrOfModulationBytes += 2;
                } // end of list marshalling
                break;
        }
        if (nrOfModulationBytes % 8 > 0) {
            int remainder = nrOfModulationBytes % 8;
            switch (remainder) {
                case 1:
                    buff.put((byte) 0);
                    buff.putShort((short) 0);
                    buff.putInt((short) 0);
                    break;
                case 2:
                    buff.putShort((short) 0);
                    buff.putInt((short) 0);
                    break;
                case 3:
                    buff.put((byte) 0);
                    buff.putInt((short) 0);
                    break;
                case 4:
                    buff.putInt((short) 0);
                    break;
                case 5:
                    buff.put((byte) 0);
                    buff.putShort((short) 0);
                    break;
                case 6:
                    buff.putShort((short) 0);
                    break;
                case 7:
                    buff.put((byte) 0);
                    break;
            }
        }
        switch (antennaPatternType) {
            case 0:
                //Omni-directional - no entries
                break;
            case 1:
                for (int idx = 0; idx < antennaPatternList.size(); idx++) {
                    BeamAntennaPattern aBeamAntennaPattern = antennaPatternList.get(idx);
                    aBeamAntennaPattern.marshal(buff);
                } // end of list marshalling
                if ((antennaPatternList.size() & 1) != 0) {//Checking for odd number of BeamAntennaPatterns and then add padding
                    buff.putInt(0xFFFFFFFF);// Padding
                }
                break;
            case 2:
                int nrOf16BitBlocks = 0;
                for (int idx = 0; idx < sphericalHarmonicAntennaPatternList.size(); idx++) {
                    SphericalHarmonicAntennaPattern aSphericalHarmonicAntennaPattern = sphericalHarmonicAntennaPatternList.get(idx);
                    aSphericalHarmonicAntennaPattern.marshal(buff);
                    int n = aSphericalHarmonicAntennaPattern.getHarmonicOrder();
                    nrOf16BitBlocks++;
                    nrOf16BitBlocks = nrOf16BitBlocks + 2 * aSphericalHarmonicAntennaPattern.getCoefficientsList().size();
                }
                if (nrOf16BitBlocks % 4 == 3) {//Missing 1 16 bit block to end on 64 bit boundry
                    buff.putShort((short) 0xFFFF);
                } else if (nrOf16BitBlocks % 4 == 1) {//missing 3 16 bit blocks to end on 64 bit boundry
                    buff.putShort((short) 0xFFFF);
                    buff.putInt(0xFFFFFFFF);
                } else if (nrOf16BitBlocks % 4 == 2) {//missing 2 16 bit blocks to end on 64 bit boundry
                    buff.putInt(0xFFFFFFFF);
                }
                break;
        } // end of list marshalling

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
        super.unmarshal(buff);

        entityId.unmarshal(buff);
        radioId = (int) (buff.getShort() & 0xFFFF);
        radioEntityType.unmarshal(buff);
        transmitState = (short) (buff.get() & 0xFF);
        inputSource = (short) (buff.get() & 0xFF);
        padding1 = (int) (buff.getShort() & 0xFFFF);
        antennaLocation.unmarshal(buff);
        relativeAntennaLocation.unmarshal(buff);
        antennaPatternType = (int) (buff.getShort() & 0xFFFF);
        antennaPatternCount = (int) (buff.getShort() & 0xFFFF);
        frequency = buff.getLong();
        transmitFrequencyBandwidth = buff.getFloat();
        power = buff.getFloat();
        modulationType.unmarshal(buff);
        cryptoSystem = (int) (buff.getShort() & 0xFFFF);
        cryptoKeyId = (int) (buff.getShort() & 0xFFFF);
        modulationParameterCount = (short) (buff.get() & 0xFF);
        padding2 = (int) (buff.getShort() & 0xFFFF);
        padding3 = (short) (buff.get() & 0xFF);

        int remainder = 0;
        int modRecordSize = 0;
        switch (getModulationType().getSystem()) {
            case 6:                                  // CCTT SINCGARS
                for (int idx = 0; idx < modulationParameterCount / 15; idx++) {
                    CcttSincgarsModulationParameters parameterRecord = new CcttSincgarsModulationParameters();
                    parameterRecord.unmarshal(buff);
                    cctSincarsModulationParametersList.add(parameterRecord);
                }
                modRecordSize = 15;
                break;
            case 8:                                  // JTIDS/MIDS
                for (int idx = 0; idx < (modulationParameterCount / 8); idx++) {
                    JtidsMidsModulationParameters parameterRecord = new JtidsMidsModulationParameters();
                    parameterRecord.unmarshal(buff);
                    jtidsMidsModulationParametersList.add(parameterRecord);
                }
                modRecordSize = 8;
                break;
            case 0:                                  // Other
            case 1:                                  // Generic
            case 2:                                  // HQ
            case 3:                                  // HQII
            case 4:                                  // HQIIA
            case 5:                                  // SINCGARS
            case 7:                                  // EPLRS
            default:                                 // Alternate radio system (Non-DIS specified)
                // Read modulation parameters byte for byte
                // The count is measured in octets, but each parameter is two octets (16 bytes)
                final int modulationParameters = modulationParameterCount / 2;

                for (int idx = 0; idx < modulationParameters; idx++) {
                    Short modulationParameter = buff.getShort();
                    modulationParametersList.add(modulationParameter);
                }
                modRecordSize = 8;
                break;
        }
        remainder = modulationParameterCount % modRecordSize;
        if (remainder > 0) {
            for (int i = 0; i < remainder; i++) {
                buff.get();//Read padding bytes
            }
        }
        switch (antennaPatternType) {
            case 0:
                break; //Omni-directional antenna pattern record
            case 1:
                for (int idx = 0; idx < antennaPatternCount / 36; idx++) {
                    BeamAntennaPattern anX = new BeamAntennaPattern();
                    anX.unmarshal(buff);
                    antennaPatternList.add(anX);
                }
                break;
            case 2:
                byte[] tmpByteArray = new byte[antennaPatternCount];
                for (int i = 0; i < antennaPatternCount; i++) {
                    tmpByteArray[i] = buff.get();
                }
                InputStream is = new ByteArrayInputStream(tmpByteArray);

                ByteBuffer buffCopy = ByteBuffer.wrap(tmpByteArray);

                DataInputStream disCopy = new DataInputStream(is);
                int recordCount = 0;
                int fieldSizeLeftOver = antennaPatternCount;
                boolean runLoop = true;
                int counter = 0;
                while (runLoop) {
                    if (fieldSizeLeftOver > 0) {
                        int harmonicOrder = tmpByteArray[counter];
                        int recordSize = ((harmonicOrder * harmonicOrder + 2 * harmonicOrder + 1) * 4) + 2;// size in bytes
                        if (fieldSizeLeftOver / recordSize >= 1 && harmonicOrder >= 0) {
                            counter = counter + recordSize;
                            fieldSizeLeftOver = fieldSizeLeftOver - recordSize;
                            recordCount++;
                        } else {
                            runLoop = false;
                        }
                    } else {
                        runLoop = false;
                    }

                }
                for (int idx = 0; idx < recordCount; idx++) {
                    SphericalHarmonicAntennaPattern anX = new SphericalHarmonicAntennaPattern();
                    anX.unmarshal(buffCopy);
                    sphericalHarmonicAntennaPatternList.add(anX);
                }
                break;
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

    @Override
    public boolean equalsImpl(Object obj) {
        boolean ivarsEqual = true;

        if (!(obj instanceof TransmitterPdu)) {
            return false;
        }

        final TransmitterPdu rhs = (TransmitterPdu) obj;

        if (!(entityId.equals(rhs.entityId))) {
            ivarsEqual = false;
        }
        if (!(radioId == rhs.radioId)) {
            ivarsEqual = false;
        }
        if (!(radioEntityType.equals(rhs.radioEntityType))) {
            ivarsEqual = false;
        }
        if (!(transmitState == rhs.transmitState)) {
            ivarsEqual = false;
        }
        if (!(inputSource == rhs.inputSource)) {
            ivarsEqual = false;
        }
        if (!(padding1 == rhs.padding1)) {
            ivarsEqual = false;
        }
        if (!(antennaLocation.equals(rhs.antennaLocation))) {
            ivarsEqual = false;
        }
        if (!(relativeAntennaLocation.equals(rhs.relativeAntennaLocation))) {
            ivarsEqual = false;
        }
        if (!(antennaPatternType == rhs.antennaPatternType)) {
            ivarsEqual = false;
        }
        if (!(antennaPatternCount == rhs.antennaPatternCount)) {
            ivarsEqual = false;
        }
        if (!(frequency == rhs.frequency)) {
            ivarsEqual = false;
        }
        if (!(transmitFrequencyBandwidth == rhs.transmitFrequencyBandwidth)) {
            ivarsEqual = false;
        }
        if (!(power == rhs.power)) {
            ivarsEqual = false;
        }
        if (!(modulationType.equals(rhs.modulationType))) {
            ivarsEqual = false;
        }
        if (!(cryptoSystem == rhs.cryptoSystem)) {
            ivarsEqual = false;
        }
        if (!(cryptoKeyId == rhs.cryptoKeyId)) {
            ivarsEqual = false;
        }
        if (!(modulationParameterCount == rhs.modulationParameterCount)) {
            ivarsEqual = false;
        }
        if (!(padding2 == rhs.padding2)) {
            ivarsEqual = false;
        }
        if (!(padding3 == rhs.padding3)) {
            ivarsEqual = false;
        }

        for (int idx = 0; idx < modulationParametersList.size(); idx++) {
            if (!(modulationParametersList.get(idx).equals(rhs.modulationParametersList.get(idx)))) {
                ivarsEqual = false;
            }
        }

        for (int idx = 0; idx < antennaPatternList.size(); idx++) {
            if (!(antennaPatternList.get(idx).equals(rhs.antennaPatternList.get(idx)))) {
                ivarsEqual = false;
            }
        }

        return ivarsEqual && super.equalsImpl(rhs);
    }
} // end of class
