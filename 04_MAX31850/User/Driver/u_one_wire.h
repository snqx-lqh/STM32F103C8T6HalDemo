#ifndef _U_ONE_WIRE_H
#define _U_ONE_WIRE_H

#include "main.h"

#include <stdint.h>
#include <stdbool.h>

// you can exclude onewire_search by defining that to 0
#ifndef ONEWIRE_SEARCH
#define ONEWIRE_SEARCH 1
#endif

// You can exclude CRC checks altogether by defining this to 0
#ifndef ONEWIRE_CRC
#define ONEWIRE_CRC 1
#endif

// Select the table-lookup method of computing the 8-bit CRC
// by setting this to 1.  The lookup table enlarges code size by
// about 250 bytes.  It does NOT consume RAM (but did in very
// old versions of OneWire).  If you disable this, a slower
// but very compact algorithm is used.
#ifndef ONEWIRE_CRC8_TABLE
#define ONEWIRE_CRC8_TABLE 1
#endif

// You can allow 16-bit CRC checks by defining this to 1
// (Note that ONEWIRE_CRC must also be 1.)
#ifndef ONEWIRE_CRC16
#define ONEWIRE_CRC16 1
#endif

typedef enum{
	ONE_WIRE_DIR_OUT = 0,
	ONE_WIRE_DIR_IN
}one_wire_dir_t;

typedef struct{
	uint8_t (*init)(void);
	uint8_t (*set_pin_dir)(one_wire_dir_t one_wire_dir);
	uint8_t (*set_pin_level)(uint8_t level);
	uint8_t (*read_pin_level)(void);
#if ONEWIRE_SEARCH
	unsigned char ROM_NO[8];
	uint8_t  LastDiscrepancy;
	uint8_t  LastFamilyDiscrepancy;
	bool     LastDeviceFlag;
#endif
}one_wire_dev_t;	

int8_t one_wire_begin(one_wire_dev_t *one_wire_dev);

// Perform a 1-Wire reset cycle. Returns 1 if a device responds
// with a presence pulse.  Returns 0 if there is no device or the
// bus is shorted or otherwise held low for more than 250uS
// 执行一次单总线复位周期。如果有设备以存在脉冲响应，则返回 1。
// 如果没有设备，或者总线短路，或者总线被拉低超过 250 微秒，则返回 0。
int8_t one_wire_reset(one_wire_dev_t *one_wire_dev);

// Issue a 1-Wire rom select command, you do the reset first.
// 发出一条单总线 ROM 选择命令，你需要先进行复位操作。
int8_t one_wire_select(one_wire_dev_t *one_wire_dev, const uint8_t rom[8]);

// Issue a 1-Wire rom skip command, to address all on bus.
// 发出一条单总线跳过 ROM 命令，以便对总线上的所有设备进行寻址。
int8_t one_wire_skip(one_wire_dev_t *one_wire_dev);

// Write a byte. If 'power' is one then the wire is held high at
// the end for parasitically powered devices. You are responsible
// for eventually depowering it by calling depower() or doing
// another read or write.
// 写入一个字节。如果 power 为 1，则在操作结束后，总线将保持高电平，
// 以便为采用寄生供电方式的设备供电。你需要通过调用 depower() 函数，
// 或者进行另一次读或写操作，最终停止对其供电。
int8_t one_wire_write(one_wire_dev_t *one_wire_dev, uint8_t v, uint8_t power);

int8_t one_wire_write_bytes(one_wire_dev_t *one_wire_dev, const uint8_t *buf, uint16_t count, bool power);

// Read a byte.
int8_t one_wire_read(one_wire_dev_t *one_wire_dev,uint8_t *read_data);

int8_t one_wire_read_bytes(one_wire_dev_t *one_wire_dev, uint8_t *buf, uint16_t count);

// Write a bit. The bus is always left powered at the end, see
// note in write() about that.
// 写入一位数据。操作结束后，总线始终保持供电状态，有关这一点可参考 write() 函数中的注释说明。
int8_t one_wire_write_bit(one_wire_dev_t *one_wire_dev, uint8_t v);

// Read a bit.
int8_t one_wire_read_bit(one_wire_dev_t *one_wire_dev,uint8_t *readV);

// Stop forcing power onto the bus. You only need to do this if
// you used the 'power' flag to write() or used a write_bit() call
// and aren't about to do another read or write. You would rather
// not leave this powered if you don't have to, just in case
// someone shorts your bus.
// 停止向总线上强制供电。仅当你在调用 write() 函数时使用了 power 标志，
// 或者调用了 write_bit() 函数，并且不打算立即进行下一次读或写操作时，
// 才需要执行此操作。如果不是必须的话，最好不要让总线一直保持供电状态，
// 以防有人意外使总线短路。
int8_t one_wire_depower(one_wire_dev_t *one_wire_dev);

#if ONEWIRE_SEARCH
// Clear the search state so that if will start from the beginning again.
// 清除搜索状态，以便能再次从起始位置开始搜索。
int8_t one_wire_reset_search(one_wire_dev_t *one_wire_dev);

// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
// 进行搜索设置，以便在下次调用 search(*newAddr) 函数时，如果存在设备类型为 family_code 的设备，就能找到该设备。
int8_t one_wire_target_search(one_wire_dev_t *one_wire_dev, uint8_t family_code);

// Look for the next device. Returns 1 if a new address has been
// returned. A zero might mean that the bus is shorted, there are
// no devices, or you have already retrieved all of them.  It
// might be a good idea to check the CRC to make sure you didn't
// get garbage.  The order is deterministic. You will always get
// the same devices in the same order.
// 查找下一个设备。如果返回了一个新的设备地址，则返回 1。返回 0 
// 可能意味着总线短路、不存在设备，或者你已经检索完所有设备。检查
// 循环冗余校验（CRC）值是个不错的办法，以确保你获取到的不是无效数
// 据。搜索顺序是确定的，你总能以相同的顺序获取到相同的设备。
int8_t one_wire_search(one_wire_dev_t *one_wire_dev, uint8_t *newAddr, bool search_mode);
#endif

#if ONEWIRE_CRC
// Compute a Dallas Semiconductor 8 bit CRC, these are used in the
// ROM and scratchpad registers.
// 计算达拉斯半导体的 8 位循环冗余校验（CRC）值，这些值用于 ROM 和暂存器寄存器中。
uint8_t crc8(const uint8_t *addr, uint8_t len);

#if ONEWIRE_CRC16
// Compute the 1-Wire CRC16 and compare it against the received CRC.
// Example usage (reading a DS2408):
//    // Put everything in a buffer so we can compute the CRC easily.
//    uint8_t buf[13];
//    buf[0] = 0xF0;    // Read PIO Registers
//    buf[1] = 0x88;    // LSB address
//    buf[2] = 0x00;    // MSB address
//    WriteBytes(net, buf, 3);    // Write 3 cmd bytes
//    ReadBytes(net, buf+3, 10);  // Read 6 data bytes, 2 0xFF, 2 CRC16
//    if (!CheckCRC16(buf, 11, &buf[11])) {
//        // Handle error.
//    }
//
// @param input - Array of bytes to checksum.
// @param len - How many bytes to use.
// @param inverted_crc - The two CRC16 bytes in the received data.
//                       This should just point into the received data,
//                       *not* at a 16-bit integer.
// @param crc - The crc starting value (optional)
// @return True, iff the CRC matches.
bool check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc);

// Compute a Dallas Semiconductor 16 bit CRC.  This is required to check
// the integrity of data received from many 1-Wire devices.  Note that the
// CRC computed here is *not* what you'll get from the 1-Wire network,
// for two reasons:
//   1) The CRC is transmitted bitwise inverted.
//   2) Depending on the endian-ness of your processor, the binary
//      representation of the two-byte return value may have a different
//      byte order than the two bytes you get from 1-Wire.
// @param input - Array of bytes to checksum.
// @param len - How many bytes to use.
// @param crc - The crc starting value (optional)
// @return The CRC16, as defined by Dallas Semiconductor.
uint16_t crc16(const uint8_t* input, uint16_t len, uint16_t crc);
#endif
#endif

#endif
