/*
 * Төмөр замын сургууль
 * Робокон 2025
 * LPMS 9-axis sensor 

  Configuration:
  1. LPMS мэдрүүрийн Swtich - ийн USBRX, USBTX, 232EN - ийг ON байрлалд бусдыг нь OFF байрлалд тохируулна.
  2. Дараах линкээр ороод OpenMAT - ийг татаж суулгана. https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1138294814/LPMS+Data+Acquisition+Software
  3. LPMS - ийг usb кабелаар компьютер руу холбоно.
  4. LPMS-Control програмыг нээгээд, + дээр дарж Scan device сонгож LPMS төхөөрөмжийн портыг хайж олоод Add device дарна.
  5. Төхөөрөмжийн портоо сонгоод, baudrate = 115200 гэж өгөөд Connect дарж холбогдоно.
  6. Төхөөрөмжийн статус дотроос Data - г expand хийгээд зөвхөн Euler - ийг сонгоод бусдыг нь идэвхгүй болгоно. Data mode - ийг нь 32bit floating point - өөр сонгоод Calibration -> Save parameter to device - ийг дарж тохиргоог хадгална.
  7. LPMS мэдрүүрийн 5VIN, GND, RX, TX - ийг arduino MEGA or DUE руу холбоод энэхүү library ашиглаж болно.
  8. LPMS - ийн дата протоколыг дараах линкээр ороод харж болно. https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1100480628/LPMS+Communication+Protocol



  Gyroscope - эргэлтийн хурд (x,y,z)
  Accelerometer - шулуун хурд (x,y,z)
  Euler - өнцөг (x,y,z)
  
  Энэхүү library нь хэвтээ тэнхлэгийн буюу Z - ийн өнцгийг (Euler) буцаадаг байхаар програмчлав. Роботын хэвтээ тэнхлэгийн дагуу өнцгийг өгнө гэсэн үг.
*/

#ifndef LPMS_H
#define LPMS_H

#include <Arduino.h>

#define MAX_PACKET_LENGTH 50
#define NOT_VALID -1000
#define X 0
#define Y 1
#define Z 2

#define HEAD 0x3A
#define END1 0x0D
#define END2 0x0A
#define COMMAND_MODE 6
#define STREAM_MODE 7
#define SET_OFFSET 18
#define GET_SENSOR_DATA 9

class LPMS {
  public:
    LPMS(HardwareSerial &hardwareSerial);
    void begin(unsigned long baudrate);
    bool requestAngle();
    bool setOffset(); // LPMS - ийн Euler_Z - ийг 0 болгоно.
    bool setMode(byte command);
    float euler[3];
    bool fail = false;

  private:
    HardwareSerial *serial;
    byte packet[50];
    uint8_t index = 0;
    bool checkLRC(uint8_t len);
    bool readPacket(unsigned long wait = 500);  
    void flush();       
};

#endif