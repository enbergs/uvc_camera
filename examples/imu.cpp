#include <iostream>

#include <fcntl.h>
#include <string.h>
#include <sys/termios.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#if 0

bool debug = false;
bool run = true;
void *anaLooper(void *ext);
void *daqLooper(void *ext);
int fdDaq;

/*** external interface ***/

#pragma pack(1)
typedef struct {
    uint8_t syncWord[2];
    uint8_t frameId;
    uint8_t frameLength;
    uint32_t timestamp;
    uint32_t serialNumber;
    uint16_t crc;
} DataFramePreamble;

#pragma pack(1)
typedef struct {
    uint16_t gyr[3];
    uint16_t acc[3];
    uint32_t sensorTime; /* one more than it should be */
} ImuData;

#pragma pack(1)
typedef struct {
    DataFramePreamble preamble;
    ImuData imuData;
} ImuDataFrame;

#pragma pack(1)
typedef struct {
    uint8_t message[100];
} GprmcData;

#pragma pack(1)
typedef struct {
    DataFramePreamble preamble;
    GprmcData gprmcData;
} GprmcDataFrame;

#define TextDataMessageSize 128
#pragma pack(1)
typedef struct {
    uint8_t message[TextDataMessageSize];
} TextData;

#pragma pack(1)
typedef struct {
    DataFramePreamble preamble;
    TextData textData;
} TextDataFrame;

enum {
    ID_UNKNOWN = 0,
    ID_HOST,
    ID_GPS,
    ID_IMU,
    ID_TEXT,
    ID_LAST
};

const size_t ImuDataFrameSize = 1;
ImuDataFrame imuDataFrame[ImuDataFrameSize];

const size_t GprmcDataFrameSize = 1;
GprmcDataFrame gprmcDataFrame[GprmcDataFrameSize];

const size_t TextDataFrameSize = 1;
TextDataFrame textDataFrame[TextDataFrameSize];

const uint8_t SyncWord0 = 0x45;
const uint8_t SyncWord1 = 0x52;

/*** end external interface ***/

uint16_t crc16Lut[256];
uint16_t crc16Mcu[256] = {
    0x0000, 0x9705, 0x2E01, 0xB904, 0x5C02, 0xCB07, 0x7203, 0xE506,
    0xB804, 0x2F01, 0x9605, 0x0100, 0xE406, 0x7303, 0xCA07, 0x5D02,
    0x7003, 0xE706, 0x5E02, 0xC907, 0x2C01, 0xBB04, 0x0200, 0x9505,
    0xC807, 0x5F02, 0xE606, 0x7103, 0x9405, 0x0300, 0xBA04, 0x2D01,
    0xE006, 0x7703, 0xCE07, 0x5902, 0xBC04, 0x2B01, 0x9205, 0x0500,
    0x5802, 0xCF07, 0x7603, 0xE106, 0x0400, 0x9305, 0x2A01, 0xBD04,
    0x9005, 0x0700, 0xBE04, 0x2901, 0xCC07, 0x5B02, 0xE206, 0x7503,
    0x2801, 0xBF04, 0x0600, 0x9105, 0x7403, 0xE306, 0x5A02, 0xCD07,
    0xC007, 0x5702, 0xEE06, 0x7903, 0x9C05, 0x0B00, 0xB204, 0x2501,
    0x7803, 0xEF06, 0x5602, 0xC107, 0x2401, 0xB304, 0x0A00, 0x9D05,
    0xB004, 0x2701, 0x9E05, 0x0900, 0xEC06, 0x7B03, 0xC207, 0x5502,
    0x0800, 0x9F05, 0x2601, 0xB104, 0x5402, 0xC307, 0x7A03, 0xED06,
    0x2001, 0xB704, 0x0E00, 0x9905, 0x7C03, 0xEB06, 0x5202, 0xC507,
    0x9805, 0x0F00, 0xB604, 0x2101, 0xC407, 0x5302, 0xEA06, 0x7D03,
    0x5002, 0xC707, 0x7E03, 0xE906, 0x0C00, 0x9B05, 0x2201, 0xB504,
    0xE806, 0x7F03, 0xC607, 0x5102, 0xB404, 0x2301, 0x9A05, 0x0D00,
    0x8005, 0x1700, 0xAE04, 0x3901, 0xDC07, 0x4B02, 0xF206, 0x6503,
    0x3801, 0xAF04, 0x1600, 0x8105, 0x6403, 0xF306, 0x4A02, 0xDD07,
    0xF006, 0x6703, 0xDE07, 0x4902, 0xAC04, 0x3B01, 0x8205, 0x1500,
    0x4802, 0xDF07, 0x6603, 0xF106, 0x1400, 0x8305, 0x3A01, 0xAD04,
    0x6003, 0xF706, 0x4E02, 0xD907, 0x3C01, 0xAB04, 0x1200, 0x8505,
    0xD807, 0x4F02, 0xF606, 0x6103, 0x8405, 0x1300, 0xAA04, 0x3D01,
    0x1000, 0x8705, 0x3E01, 0xA904, 0x4C02, 0xDB07, 0x6203, 0xF506,
    0xA804, 0x3F01, 0x8605, 0x1100, 0xF406, 0x6303, 0xDA07, 0x4D02,
    0x4002, 0xD707, 0x6E03, 0xF906, 0x1C00, 0x8B05, 0x3201, 0xA504,
    0xF806, 0x6F03, 0xD607, 0x4102, 0xA404, 0x3301, 0x8A05, 0x1D00,
    0x3001, 0xA704, 0x1E00, 0x8905, 0x6C03, 0xFB06, 0x4202, 0xD507,
    0x8805, 0x1F00, 0xA604, 0x3101, 0xD407, 0x4302, 0xFA06, 0x6D03,
    0xA004, 0x3701, 0x8E05, 0x1900, 0xFC06, 0x6B03, 0xD207, 0x4502,
    0x1800, 0x8F05, 0x3601, 0xA104, 0x4402, 0xD307, 0x6A03, 0xFD06,
    0xD007, 0x4702, 0xFE06, 0x6903, 0x8C05, 0x1B00, 0xA204, 0x3501,
    0x6803, 0xFF06, 0x4602, 0xD107, 0x3401, 0xA304, 0x1A00, 0x8D05
};

uint16_t crc16Polynomial = 0x8005;

void makeCrc16Table(uint16_t polynomial, uint16_t *table) {
    for (unsigned int i = 0; i < 256; i++) {
        uint16_t crc = i;

        for (unsigned int j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (-(int16_t) (crc & 1) & polynomial);
        table[i] = crc;
        if ((i % 8) == 0) { printf("\n"); }
        printf("%4.4x, ", crc);
        /* my_printf("crc[%u] 0x%x\r\n", i, crc); */
    }
    printf("\n");
}

void initializeCrc(void) {
    makeCrc16Table(crc16Polynomial, crc16Lut);
    for (unsigned int i = 0; i < 256; ++i) {
        if (crc16Lut[i] != crc16Mcu[i]) {
            printf("mismatch between mcu and host. index = i. entries = %2.2x vs %2.2x\n", crc16Lut[i], crc16Mcu[i]);
        }
    }
}

uint16_t crc16(const uint8_t *p, unsigned int len) {
    uint16_t crc = ~0;

    while (len--) {
        uint8_t ch1 = *p++;
        uint8_t ch2 = crc >> 8;
        uint16_t tCrc = crc << 8;
        crc = tCrc ^ crc16Lut[ch2 ^ ch1];
    }
    return ~crc;
}

int reverse(uint16_t xIn) {
    int16_t xOut = ((xIn & 0xff) << 8) | ((xIn >> 8) & 0xff);
    // if (debug) { printf("compare %4.4x to %4.4x\n", xIn, xOut); }
    return xOut;
}

typedef struct {
    std::string command;
    int pause;
} AtCommand;

int configureSerialPort(int fd, int baudRate, bool canonical = true, int min_chars = 0) {
    struct termios settings;
    tcgetattr(fd, &settings);
    speed_t baud = 0;
    switch (baudRate) {
    case 921600:
        baud = B921600;
        break;
    case 230400:
        baud = B230400;
        break;
    case 115200:
        baud = B115200;
        break;
    }

    // int cfmakeraw( struct termios * termios_p );

    cfmakeraw(&settings);

    cfsetospeed(&settings, baud); /* baud rate */

#if 0
    termios_p->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
    termios_p->c_oflag &= ~OPOST;
    termios_p->c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
    termios_p->c_cflag &= ~(CSIZE|PARENB);
    termios_p->c_cflag |= CS8;

    settings.c_cflag &= ~PARENB;  /* no parity */
    settings.c_cflag &= ~CSTOPB;  /* 1 stop bit */
    settings.c_cflag &= ~CSIZE;
    settings.c_cflag |= (CLOCAL | CREAD);
    settings.c_lflag = canonical ? ICANON : 0;
    settings.c_oflag &= ~OPOST;        /* raw output */
    settings.c_cc[VMIN] = min_chars;
    settings.c_cc[VTIME] = 1; /* 100ms timeout */
#endif

    tcsetattr(fd, TCSANOW, &settings); /* apply settings */
    tcflush(fd, TCIOFLUSH);
    return 0;
}

enum {
    TestModeDataFlow = 0,
    TestModePulse,
    TestModes
};

uint8_t **triggerBoardDataBuff;
unsigned int *triggerBoardDataFrameLength;
unsigned int triggerBoardDataFrameSize = 1024;
unsigned int triggerBoardDataSize = 32;
unsigned int triggerBoardDataHead = 0;
unsigned int triggerBoardDataTail = 0;
unsigned int triggerBoardDataMask = 31;

int main(int argc, char **argv) {
    pthread_t anaThreadId, daqThreadId;
    int baudRate = 115200, minChars = sizeof(ImuDataFrameSize);
    const char *devName = "/dev/ttyUSB0";
    std::vector<AtCommand> atCommands;
    int testMode = TestModeDataFlow;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-d") == 0) {
            devName = argv[++i];
        } else if (strcmp(argv[i], "-b") == 0) {
            baudRate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-test") == 0) {
            testMode = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-min-chars") == 0) {
            minChars = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-at") == 0) {
            AtCommand atCommand;
            atCommand.command = argv[++i];
            atCommand.pause = atoi(argv[++i]);
            atCommands.push_back(atCommand);
        } else if (strcmp(argv[i], "-buff") == 0) {
            triggerBoardDataFrameSize = atoi(argv[++i]);
            triggerBoardDataSize = atoi(argv[++i]);
            triggerBoardDataMask = triggerBoardDataSize - 1;
        } else if (strcmp(argv[i], "-debug") == 0) {
            debug = true;
        }
    }

    triggerBoardDataBuff = new uint8_t * [triggerBoardDataSize];
    triggerBoardDataFrameLength = new unsigned int [triggerBoardDataSize];
    for (unsigned int i = 0; i < triggerBoardDataSize; ++i) {
        triggerBoardDataBuff[i] = new uint8_t [triggerBoardDataFrameSize];
        triggerBoardDataFrameLength[i] = 0;
    }

    bool canonical = false;
    fdDaq = open(devName, O_RDWR | O_NOCTTY);
    configureSerialPort(fdDaq, baudRate, canonical, minChars);

    int err;
    unsigned int cpuMask;
    cpu_set_t cpu_set;

    err = pthread_create(&anaThreadId, NULL, anaLooper, NULL); /* create thread */
    cpuMask = 1;
    CPU_ZERO(&cpu_set);
    for (int i = 0; i < 32; ++i) { if (cpuMask & (1 << i)) CPU_SET(i, &cpu_set); }
    err = pthread_setaffinity_np(anaThreadId, sizeof(cpu_set_t), &cpu_set);

    err = pthread_create(&daqThreadId, NULL, daqLooper, NULL); /* create thread */
    cpuMask = 2;
    CPU_ZERO(&cpu_set);
    for (int i = 0; i < 32; ++i) { if (cpuMask & (1 << i)) CPU_SET(i, &cpu_set); }
    err = pthread_setaffinity_np(daqThreadId, sizeof(cpu_set_t), &cpu_set);

    initializeCrc();

    const char *pulseCommand = "$ATPULSE*";
    while (testMode == TestModePulse) {
        write(fdDaq, pulseCommand, strlen(pulseCommand));
        sleep(1);
    }

    std::vector<AtCommand>::const_iterator atCommandIt, lastAtCommand = atCommands.end();
    for (atCommandIt = atCommands.begin(); atCommandIt != lastAtCommand; ++atCommandIt) {
        int pause = atCommandIt->pause;
        printf("issue command [%s] with pause = [%d]\n", atCommandIt->command.c_str(), pause);
        write(fdDaq, atCommandIt->command.c_str(), atCommandIt->command.length());
        if (pause != 0) { sleep(pause); }
    }

    while (run) {}

    return 0;
}

void *daqLooper(void *ext) {
    while (run) {
        int preincrementedHead = (triggerBoardDataHead + 1) % triggerBoardDataSize;
        if (preincrementedHead == triggerBoardDataTail) {
            usleep(100);
        } else {
            uint8_t *buffer = triggerBoardDataBuff[triggerBoardDataHead];
            int nRead = read(fdDaq, buffer, triggerBoardDataFrameSize);
            if (nRead > 0) {
                triggerBoardDataFrameLength[triggerBoardDataHead] = nRead;
                triggerBoardDataHead = (triggerBoardDataHead + 1) % triggerBoardDataSize;
            } else {
                usleep(100);
            }
        }
    }
    return 0;
}

void *anaLooper(void *ext) {
    int state = 0;
//    constexpr size_t bufferSize = 1024;
//    uint8_t buffer[bufferSize];
    uint8_t *stateRxBuffer;
    int counter = 0, stateCounter;
    bool isImu, isGps, isTxt;
    time_t time0 = time(0);
    static time_t previousTime = 0;
    DataFramePreamble *preamble;
    unsigned int frameLength;
    uint16_t lastImuSerialNumber = 0, lastGpsSerialNumber = 0, lastTextSerialNumber = 0;
    while (run) {
        if (triggerBoardDataHead == triggerBoardDataTail) {
            usleep(100);
            continue;
        }
        const uint8_t *buffer = triggerBoardDataBuff[triggerBoardDataTail];
        unsigned int nRead = triggerBoardDataFrameLength[triggerBoardDataTail];
//        printf("nRead = %d H=%d/T=%d\n", nRead, triggerBoardDataHead, triggerBoardDataTail);
        triggerBoardDataTail = (triggerBoardDataTail + 1) % triggerBoardDataSize;
        for (unsigned int i = 0; i < nRead; ++i) {
            uint8_t byte = buffer[i];
            switch (state) {
            case 0:
                if (byte == SyncWord0) {
                    state = 1;
                } else {
                    ++stateCounter;
                    if (debug) {
                        printf("syncing byte = %2.2x counter = %d\n", byte, stateCounter);
                    }
                }
                break;

            case 1:
                if (byte == SyncWord1) {
                    state = 2;
                } else {
                    if (debug) {
                        printf("false alarm. return to sync...\n");
                    }
                    state = 0;
                }
                break;

            case 2:
                if (byte == ID_IMU) {
                    isImu = true;
                    preamble = &imuDataFrame->preamble;
                    stateCounter = sizeof(DataFramePreamble);
                    stateRxBuffer = (uint8_t *) imuDataFrame;
                    *stateRxBuffer++ = SyncWord0; --stateCounter;
                    *stateRxBuffer++ = SyncWord1; --stateCounter;
                    *stateRxBuffer++ = byte; --stateCounter;
                    state = 3;
                } else if (byte == ID_GPS) {
                    isGps = true;
                    preamble = &gprmcDataFrame->preamble;
                    stateCounter = sizeof(DataFramePreamble);
                    stateRxBuffer = (uint8_t *) gprmcDataFrame;
                    *stateRxBuffer++ = SyncWord0; --stateCounter;
                    *stateRxBuffer++ = SyncWord1; --stateCounter;
                    *stateRxBuffer++ = byte; --stateCounter;
                    state = 3;
                } else if (byte == ID_TEXT) {
                    isTxt = true;
                    preamble = &textDataFrame->preamble;
                    stateCounter = sizeof(DataFramePreamble);
                    stateRxBuffer = (uint8_t *) textDataFrame;
                    *stateRxBuffer++ = SyncWord0; --stateCounter;
                    *stateRxBuffer++ = SyncWord1; --stateCounter;
                    *stateRxBuffer++ = byte; --stateCounter;
                    state = 3;
                } else {
                    if (debug) {
                        printf("false alarm. return to sync...\n");
                    }
                    state = 0;
                }
                break;

            case 3:
                *stateRxBuffer++ = byte; --stateCounter;
                if (stateCounter == 0) {
                    frameLength = preamble->frameLength;
                    if (frameLength < sizeof(DataFramePreamble)) {
                        state = 0; /* bad */
                    } else {
                        stateCounter = frameLength - sizeof(DataFramePreamble); /* how much we've gathered so far */
                        state = 4;
                    }
                }
                break;

            case 4:
                *stateRxBuffer++ = byte; --stateCounter;
                if (stateCounter == 0) {
                    if (isImu && (frameLength == sizeof(ImuDataFrame))) {

                        ++counter;

#if 0
                        uint8_t *p = (uint8_t *) imuDataFrame;
                        printf("%2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n",
                            p[0x00], p[0x01], p[0x02], p[0x03], p[0x04], p[0x05], p[0x06], p[0x07],
                            p[0x08], p[0x09], p[0x0a], p[0x0b], p[0x0c], p[0x0d], p[0x0e], p[0x0f],
                            p[0x10], p[0x11], p[0x12], p[0x13], p[0x14], p[0x15], p[0x16], p[0x17],
                            p[0x18], p[0x19], p[0x1a], p[0x1b]);
#endif

                        uint16_t incomingCrc = preamble->crc;
                        preamble->crc = 0;
                        uint16_t crc = crc16((uint8_t *) preamble, preamble->frameLength);
                        preamble->crc = incomingCrc;
                        if (crc == incomingCrc) {
                            int16_t accX = imuDataFrame->imuData.acc[0];
                            int16_t accY = imuDataFrame->imuData.acc[1];
                            int16_t accZ = imuDataFrame->imuData.acc[2];
                            int16_t gyrX = imuDataFrame->imuData.gyr[0];
                            int16_t gyrY = imuDataFrame->imuData.gyr[1];
                            int16_t gyrZ = imuDataFrame->imuData.gyr[2];
                            float aX = (2.0 * 9.8 * (accX / 32768.0));
                            float aY = (2.0 * 9.8 * (accY / 32768.0));
                            float aZ = (2.0 * 9.8 * (accZ / 32768.0));
                            float gX = (125.0 * (gyrX / 32768.0));
                            float gY = (125.0 * (gyrY / 32768.0));
                            float gZ = (125.0 * (gyrZ / 32768.0));
                            static uint32_t previousTimestamp = 0;
                            uint32_t reportedTime = imuDataFrame->preamble.timestamp - previousTimestamp;
                            if (preamble->serialNumber != (lastImuSerialNumber + 1)) {
                                printf("imu serial number out of sequence. %d vs %d\n", preamble->serialNumber, lastImuSerialNumber);
                            }
                            lastImuSerialNumber = preamble->serialNumber;

#if 1
                            printf("T = %8.8x/%8.8x/%8.8x S=%4.4x ACC = %8.2f %8.2f %8.2f GYR = %8.2f %8.2f %8.2f\n",
                                reportedTime, preamble->timestamp, imuDataFrame->imuData.sensorTime, preamble->serialNumber, aX, aY, aZ, gX, gY, gZ);
#endif

                            previousTimestamp = imuDataFrame->preamble.timestamp;
                            time_t currentTime = time(0);
                            if (currentTime != previousTime) {
                                time_t elapsedTime = currentTime - time0;
                                float sampleRate = counter;
                                sampleRate /= elapsedTime;
//                            printf("rate = %f = %d / %ld\n", sampleRate, counter, elapsedTime);
                            }
                            previousTime = currentTime;
                        } else {
                            static int lastSerialNumber = 0;
                            int n = imuDataFrame->preamble.serialNumber - lastSerialNumber;
                            lastSerialNumber = imuDataFrame->preamble.serialNumber;
                            printf("imu -> wtf, man! %8.8x vs %8.8x. %d samples between\n", crc, incomingCrc, n);
                        }
                    } else if ((isGps == true)  && (frameLength <= sizeof(GprmcDataFrame))) {
                        static uint32_t previousTime = 0;

                        uint16_t incomingCrc = preamble->crc;
                        preamble->crc = 0;
                        uint16_t crc = crc16((uint8_t *) preamble, preamble->frameLength);
                        preamble->crc = incomingCrc;
                        if (crc == incomingCrc) {
                            uint32_t reportedTime = gprmcDataFrame->preamble.timestamp - previousTime;
                            previousTime = gprmcDataFrame->preamble.timestamp;
                            if (preamble->serialNumber != (lastGpsSerialNumber + 1)) {
                                printf("gps serial number out of sequence. %d vs %d\n", preamble->serialNumber, lastGpsSerialNumber);
                            }
                            lastGpsSerialNumber = preamble->serialNumber;
                            printf("T = %8.8x/%8.8x/%d POSITION = [%s]\n", reportedTime, gprmcDataFrame->preamble.timestamp,
                                preamble->serialNumber, gprmcDataFrame->gprmcData.message);
                        } else {
#if 1
                            uint8_t *p = (uint8_t *) gprmcDataFrame;
                            printf("%2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n",
                                p[0x00], p[0x01], p[0x02], p[0x03], p[0x04], p[0x05], p[0x06], p[0x07],
                                p[0x08], p[0x09], p[0x0a], p[0x0b], p[0x0c], p[0x0d], p[0x0e], p[0x0f],
                                p[0x10], p[0x11], p[0x12], p[0x13], p[0x14], p[0x15], p[0x16], p[0x17],
                                p[0x18], p[0x19], p[0x1a], p[0x1b], p[0x1c], p[0x1d], p[0x1e], p[0x1f],
                                p[0x20], p[0x21], p[0x22], p[0x23], p[0x24], p[0x25], p[0x26], p[0x27],
                                p[0x28], p[0x29], p[0x2a], p[0x2b], p[0x2c], p[0x2d], p[0x2e], p[0x2f]
                            );
#endif
                            printf("gps -> wtf, man! %8.8x vs %8.8x\n", crc, incomingCrc);
                        }
                    } else if ((isTxt == true)  && (frameLength <= sizeof(TextDataFrame))) {
                        uint16_t incomingCrc = preamble->crc;
                        preamble->crc = 0;
                        uint16_t crc = crc16((uint8_t *) preamble, preamble->frameLength);
                        if (crc == incomingCrc) {
                            printf("T = %8.8x TEXT=[%s]\n", preamble->timestamp, textDataFrame->textData.message);
                        } else {
                            printf("txt -> wtf, man! %8.8x vs %8.8x\n", crc, preamble->crc);
                        }
                    } else {
                    }
                    state = 0;
                    stateCounter = 0;
                }
                break;

            default:
                printf("unknown/unexpected state\n");
                state = 0;
                break;
            }
        }
    }
    return 0;
}

#endif

