/*
 * Frame.hpp
 *
 *  Created on: 22 Apr 2023
 *      Author: rw123
 */

#ifndef FRAME_HPP_
#define FRAME_HPP_

#include <stdint.h>

namespace SerLink
{

class Frame{
public:
  static const char TYPE_TRANSMISSION = 'T';
  static const char TYPE_UNIDIRECTION = 'U';
  static const char TYPE_ACK = 'A';
  static const int LEN_PROTOCOL = 5;
  static const int LEN_TYPE = 1;
  static const int LEN_ROLLCODE = 3;
  static const int LEN_DATALEN = 3;
  static const int LEN_HEADER = LEN_PROTOCOL + LEN_TYPE + LEN_ROLLCODE + LEN_DATALEN;
  static const int INDEX_START_PROTOCOL = 0;
  static const int INDEX_START_TYPE = LEN_PROTOCOL;
  static const int INDEX_START_ROLLCODE = INDEX_START_TYPE + LEN_TYPE;
  static const int INDEX_START_DATALEN = INDEX_START_ROLLCODE + LEN_ROLLCODE;
  static const int INDEX_START_DATA = INDEX_START_DATALEN + LEN_DATALEN;
  static const int MAX_DATALEN = 64;
  static const int MAX_FRAME_LEN = LEN_HEADER + MAX_DATALEN + 2; // +2 for CR/LF

  // DataLen (Ack code) return codes
  static const int ACK_OK = 900;

  char protocol[LEN_PROTOCOL];
  char type;
  uint16_t rollCode;
  uint16_t dataLen;
  char data[MAX_DATALEN];

  Frame();
  Frame(char* pProtocol, char type, uint16_t rollCode, uint16_t dataLen, char* pData);
  void setProtocol(char* pProtocol);

  // Sets dataLen and copies pData into this frame's own data buffer (zeroed
  // first, then clamped to the buffer's fixed size). No copy happens if
  // dataLen is being used as an ack status code (>= ACK_OK).
  void setData(uint16_t dataLen, char* pData);
  void toString(char* const pStr, uint8_t* pRetCode);
  void copy(Frame* copy);
  void toDebugString(char* pStr);
  static void incRollCode(uint16_t* rollCode);

  // Increments this frame's own rollCode (with the same wraparound as the
  // static overload above).
  void incRollCode();
  static void fromString(char* str, Frame* pFrame);
  static void int3dToStr(const uint16_t rollCode, char* pStr);
  static void str3dToInt(char* str, uint16_t* value);

private:

    static uint8_t digitCharValue(char c);
};

}
#endif /* FRAME_HPP_ */
