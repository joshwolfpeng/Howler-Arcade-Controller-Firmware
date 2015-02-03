// E:\arcadeCode\HowlerController\HowlerController\src\Desc1.h


char ReportDescriptor[54] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x32,                    //     USAGE (Z)
    0x15, 0x9c,                    //     LOGICAL_MINIMUM (-100)
    0x25, 0x64,                    //     LOGICAL_MAXIMUM (100)
    0x35, 0xff,                    //     PHYSICAL_MINIMUM (-1)
    0x45, 0x01,                    //     PHYSICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //     END_COLLECTION
    0x05, 0x09,                    //   USAGE_PAGE (Button)
    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
    0x29, 0x0f,                    //   USAGE_MAXIMUM (Button 15)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x0f,                    //   REPORT_COUNT (15)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x01,                    //   INPUT (Cnst,Ary,Abs)
    0xc0                           // END_COLLECTION
};

