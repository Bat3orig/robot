#include "lpms.h"

// 4 byte - ийг float руу хөрвүүлэх union
union {
  byte bytes[4];
  float value;
} floatUnion;

LPMS::LPMS(HardwareSerial &hardwareSerial) {
  this->serial = &hardwareSerial;  
}

void LPMS::begin(unsigned long baudrate) {
  this->serial->begin(baudrate);
}

bool LPMS::requestAngle() {
  // LPMS мэдрүүр дээр тохируулсан sensor - уудын датаг авах хүсэлтийн packet
  byte buffer[11] = {HEAD, 0x01, 0x00, GET_SENSOR_DATA, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x0D, 0x0A};

  flush();  // serial -> rx buffer хоослох.

  for(uint8_t i = 0; i < 11; i++)
    serial->write(buffer[i]);

  // LPMS - ээс ирсэн хариуг унших
  bool ret = readPacket(50);
  if(ret) {
    // Хариу ирсэн бол EULER-Z - ийг float руу хөрвүүлнэ.
    // packet - ийн 11,12,13,14 индексүүд дээр X тэнхлэгийн өнцөг байгаа.
    // packet - 15,16,17,18 индексүүд дээр Y тэнхлэгийн өнцөг байгаа.
    // packet - 19,20,21,22 индексүүд дээр Z тэнхлэгийн өнцөг байгаа.
    floatUnion.bytes[0] = packet[19];
    floatUnion.bytes[1] = packet[20];
    floatUnion.bytes[2] = packet[21];
    floatUnion.bytes[3] = packet[22];
    // Өгөгдсөн өнцөг нь радиан тул үүнийг degree буюу өнцөг рүү хөрвүүлж буцаана.
    this->euler[Z] = floatUnion.value * RAD_TO_DEG;
    return true;
  }
  return false;
}

// LPMS - ээс ирэх packet - ийг уншиж, шалгах функц. Өгөгдсөн wait (миллисекунд) хугацаанд өгөгдөл ирэхгүй бол parse хийхгүй.
bool LPMS::readPacket(unsigned long wait){
    unsigned long current = millis();
    bool ready = false;
    while(1) {
      if(millis() - current > wait) 
        return false;
      if(serial->available()) {
        byte Byte= serial->read();
        // Packet толгой буюу HEAD ирээгүй бол
        if(!ready) {
          if(Byte == HEAD) {
            ready = true;
            index = 0;
          }
        }
        // Packet толгой HEAD ирсэн бол үргэлжлүүлэн parse хийнэ.
        if(ready) {
          if(index < MAX_PACKET_LENGTH)
            packet[index++] = Byte;
          else 
            return false;
          // Хэрвээ packet - ийн END байтууд ирсэн бол packet - ийн LRC - ийг шалгаад буцаана.
          if(packet[index - 2] == END1 && packet[index - 1] == END2) {
            ready = false;
            return checkLRC(index-3);
          }
        }
      }
    }
    return false;
}

bool LPMS::setOffset() {
  // Euler өнцгийг 0 болгох хүсэлтийн packet
  byte buffer[15] = {HEAD, 0x01, 0x00, SET_OFFSET, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 23, 0x00, END1, END2};
  flush();
  for(uint8_t i = 0; i < 15; i++) {
    serial->write(buffer[i]);
  }
  bool ret = readPacket(500);
  if(ret) {
    if(packet[3] == 0 && packet[4] == 0) 
      return true;
  }
  return false;
}

bool LPMS::setMode(byte command) {
  byte lrc = 1 + command;
  byte buffer[11] = {HEAD, 0x01, 0x00, command, 0x00, 0x00, 0x00, lrc, 0x00, END1, END2};

  flush();

  for(uint8_t i = 0; i < 11; i++) {
    serial->write(buffer[i]);
  }

  bool ret = readPacket(500);
  if(ret) {
    if(packet[3] == 0 && packet[4] == 0) 
      return true;
  }
  return false;
}

bool LPMS::checkLRC(uint8_t len){
  int16_t LRC  = (packet[len] << 8) | (packet[len-1]);
  int16_t lrc = 0;
  for(int i = 1; i < len-1; i++)
    lrc += packet[i];
  if(lrc == LRC)
    return true;
  else 
    return false;
}

// clear buffer
void LPMS::flush() {
  while(serial->available())
    serial->read();
}