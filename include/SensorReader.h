class SensorReader {
    public:
        SensorReader(int sensorL, int sensorM, int sensorR);

        int sensorL, sensorM, sensorR;
        bool sensorLval, sensorMval, sensorRval;

        void readSensors();
};