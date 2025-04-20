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
// ִ��һ�ε����߸�λ���ڡ�������豸�Դ���������Ӧ���򷵻� 1��
// ���û���豸���������߶�·���������߱����ͳ��� 250 ΢�룬�򷵻� 0��
int8_t one_wire_reset(one_wire_dev_t *one_wire_dev);

// Issue a 1-Wire rom select command, you do the reset first.
// ����һ�������� ROM ѡ���������Ҫ�Ƚ��и�λ������
int8_t one_wire_select(one_wire_dev_t *one_wire_dev, const uint8_t rom[8]);

// Issue a 1-Wire rom skip command, to address all on bus.
// ����һ������������ ROM ����Ա�������ϵ������豸����Ѱַ��
int8_t one_wire_skip(one_wire_dev_t *one_wire_dev);

// Write a byte. If 'power' is one then the wire is held high at
// the end for parasitically powered devices. You are responsible
// for eventually depowering it by calling depower() or doing
// another read or write.
// д��һ���ֽڡ���� power Ϊ 1�����ڲ������������߽����ָߵ�ƽ��
// �Ա�Ϊ���ü������緽ʽ���豸���硣����Ҫͨ������ depower() ������
// ���߽�����һ�ζ���д����������ֹͣ���乩�硣
int8_t one_wire_write(one_wire_dev_t *one_wire_dev, uint8_t v, uint8_t power);

int8_t one_wire_write_bytes(one_wire_dev_t *one_wire_dev, const uint8_t *buf, uint16_t count, bool power);

// Read a byte.
int8_t one_wire_read(one_wire_dev_t *one_wire_dev,uint8_t *read_data);

int8_t one_wire_read_bytes(one_wire_dev_t *one_wire_dev, uint8_t *buf, uint16_t count);

// Write a bit. The bus is always left powered at the end, see
// note in write() about that.
// д��һλ���ݡ���������������ʼ�ձ��ֹ���״̬���й���һ��ɲο� write() �����е�ע��˵����
int8_t one_wire_write_bit(one_wire_dev_t *one_wire_dev, uint8_t v);

// Read a bit.
int8_t one_wire_read_bit(one_wire_dev_t *one_wire_dev,uint8_t *readV);

// Stop forcing power onto the bus. You only need to do this if
// you used the 'power' flag to write() or used a write_bit() call
// and aren't about to do another read or write. You would rather
// not leave this powered if you don't have to, just in case
// someone shorts your bus.
// ֹͣ��������ǿ�ƹ��硣�������ڵ��� write() ����ʱʹ���� power ��־��
// ���ߵ����� write_bit() ���������Ҳ���������������һ�ζ���д����ʱ��
// ����Ҫִ�д˲�����������Ǳ���Ļ�����ò�Ҫ������һֱ���ֹ���״̬��
// �Է���������ʹ���߶�·��
int8_t one_wire_depower(one_wire_dev_t *one_wire_dev);

#if ONEWIRE_SEARCH
// Clear the search state so that if will start from the beginning again.
// �������״̬���Ա����ٴδ���ʼλ�ÿ�ʼ������
int8_t one_wire_reset_search(one_wire_dev_t *one_wire_dev);

// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
// �����������ã��Ա����´ε��� search(*newAddr) ����ʱ����������豸����Ϊ family_code ���豸�������ҵ����豸��
int8_t one_wire_target_search(one_wire_dev_t *one_wire_dev, uint8_t family_code);

// Look for the next device. Returns 1 if a new address has been
// returned. A zero might mean that the bus is shorted, there are
// no devices, or you have already retrieved all of them.  It
// might be a good idea to check the CRC to make sure you didn't
// get garbage.  The order is deterministic. You will always get
// the same devices in the same order.
// ������һ���豸�����������һ���µ��豸��ַ���򷵻� 1������ 0 
// ������ζ�����߶�·���������豸���������Ѿ������������豸�����
// ѭ������У�飨CRC��ֵ�Ǹ�����İ취����ȷ�����ȡ���Ĳ�����Ч��
// �ݡ�����˳����ȷ���ģ�����������ͬ��˳���ȡ����ͬ���豸��
int8_t one_wire_search(one_wire_dev_t *one_wire_dev, uint8_t *newAddr, bool search_mode);
#endif

#if ONEWIRE_CRC
// Compute a Dallas Semiconductor 8 bit CRC, these are used in the
// ROM and scratchpad registers.
// �������˹�뵼��� 8 λѭ������У�飨CRC��ֵ����Щֵ���� ROM ���ݴ����Ĵ����С�
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
