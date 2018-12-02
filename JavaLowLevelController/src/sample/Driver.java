package sample;

import com.fazecast.jSerialComm.SerialPort;

class Driver {

    int stretchMin = -180;
    int stretchMax = 180;

    int rotationMin = -180;
    int rotationMax = 180;

    int heightMin = -180;
    int heightMax = 180;

    int handrotMin = -180;
    int handrotMax = 180;

    int stretch = 0;
    int rotation = 0;
    int height = 0;
    int handrot = 0;
    boolean grip = false;
    boolean gripDirection;

    private SerialPort serial;
    byte[] buffer = new byte[12];

    Driver() {
        serial = SerialPort.getCommPort("/dev/cu.wchusbserial1410");
        serial.setBaudRate(9600);
        serial.openPort();
    }

    public void sendData() {
        System.out.println(String.format("q1 %d q2 %d q3 %d \n", rotation, stretch, height));
        try {
            buffer[0] = (byte) 0xFF;
            buffer[1] = (byte) 0xAA;
            if (rotation > 0) {
                buffer[2] = (byte)0xFF;
                buffer[3] = (byte) (255 - rotation);
            } else {
                buffer[2] = (byte)0x00;
                buffer[3] = (byte) -rotation;
            }

            if (stretch > 0) {
                buffer[4] = (byte)0xFF;
                buffer[5] = (byte) (255 - stretch);
            } else {
                buffer[4] = (byte)0x00;
                buffer[5] = (byte) -stretch;
            }

            if (height > 0) {
                buffer[6] = (byte)0xFF;
                buffer[7] = (byte) (255 - height);
            } else {
                buffer[6] = (byte)0x00;
                buffer[7] = (byte) -height;
            }

            if (handrot > 0) {
                buffer[8] = (byte)0xFF;
                buffer[9] = (byte) (255 - handrot);
            } else {
                buffer[8] = (byte)0x00;
                buffer[9] = (byte) -handrot;
            }

            if (grip) {
                System.out.println(gripDirection);
                if (gripDirection) {
                    buffer[10] = (byte)0x01;
                } else {
                    buffer[10] = (byte)0x10;
                }
            } else {
                buffer[10] = 0x00;
            }

            buffer[11] = 0x00; // TODO will be checksum
            serial.writeBytes(buffer, 12);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    public void setStretch(int stretch) {
        if (stretch <= stretchMax && stretch >= stretchMin){
            this.stretch = stretch;
        }
    }

    public void setRotation(int rotation) {
        if (rotation <= rotationMax && rotation >= rotationMin){
            this.rotation = rotation;
        }
    }

    public void setHeight(int height) {
        if (height <= heightMax && height >= heightMin)
        this.height = height;
    }

    public void setHandrot(int handrot) {
        if (handrot <= handrotMax && handrot >= handrotMin)
        this.handrot = handrot;
    }

    public void setGrip(boolean grip, boolean gripDirection) {
        this.grip = grip;
        this.gripDirection = gripDirection;
    }

    public void setJointAngles(double q1, double q2, double q3) {
        int q1_degree = (int)Math.round(q1 / Math.PI * 180);
        int q2_degree = (int)Math.round((q2 - Math.PI / 2)/ Math.PI * 180) + 6;
        int q3_degree = (int)Math.round(q3 / Math.PI * 180) - 48;

        this.setRotation(q1_degree);
        this.setStretch(q2_degree);
        this.setHeight(q3_degree);
    }
}
