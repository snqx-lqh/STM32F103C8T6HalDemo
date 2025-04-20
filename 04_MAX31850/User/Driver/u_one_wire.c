#include "u_one_wire.h"

/**************�û����������*****************/
#include "delay.h"

#define noInterrupts()         __disable_irq();  //ʧ�ܵ�Ƭ���ж�
#define interrupts()           __enable_irq();   //ʹ�ܵ�Ƭ���ж�
#define one_wire_delay_us      DWT_Delay_us
/********************************************/

/**
  * @brief   ��ʼ��������Ҫ�������ų�ʼ�����Լ����ʹ����������ַ������Ӧ�����ݱ�������
  * @param   one_wire_dev��one_wire�����豸����ָ�� 
  * @retval  -1�������豸δ���� -2�������豸δ����  0����ʼ���ɹ�
 **/
int8_t one_wire_begin(one_wire_dev_t *one_wire_dev)
{
	int8_t ret;
	
	if(one_wire_dev == NULL)
		return -1;
	
	// ������ų�ʼ��
	one_wire_dev->init();
	
	#if ONEWIRE_SEARCH
		ret = one_wire_reset_search(one_wire_dev);
		if(ret == -1) return -2; 
	#endif
	
	return 0;
}

/**
  * @brief   ��λ��������׼������������ת���Ŀ�ʼ�ź�
  * @param   one_wire_dev��one_wire�����豸����ָ�� 
  * @retval  -1�������豸δ���� 0:���豸 1�����豸 
 **/
int8_t one_wire_reset(one_wire_dev_t *one_wire_dev)
{
	int8_t read_data;
	uint8_t retries = 125;
	
	if(one_wire_dev == NULL)
		return -1;
	
	noInterrupts();
	one_wire_dev->set_pin_dir(ONE_WIRE_DIR_IN);
    interrupts();
    // wait until the wire is high... just in case
    do {
        if (--retries == 0) return 0;
        one_wire_delay_us(2);
    } while ( !one_wire_dev->read_pin_level());
    noInterrupts();
	one_wire_dev->set_pin_level(0);
	one_wire_dev->set_pin_dir(ONE_WIRE_DIR_OUT);
    interrupts();
    one_wire_delay_us(480);
    noInterrupts();
	one_wire_dev->set_pin_dir(ONE_WIRE_DIR_IN);
    one_wire_delay_us(70);
    read_data = !one_wire_dev->read_pin_level();
    interrupts();
    one_wire_delay_us(410);
    return read_data;

}

/**
  * @brief   ������д1λ������
  * @param   one_wire_dev��one_wire�����豸����ָ��  v��Ҫд�������
  * @retval  -1�������豸δ����  0���ɹ�
 **/
int8_t one_wire_write_bit(one_wire_dev_t *one_wire_dev,uint8_t v)
{
	if(one_wire_dev == NULL)
		return -1;
	
	if (v & 1) {
        noInterrupts();
        one_wire_dev->set_pin_level(0);
        one_wire_dev->set_pin_dir(ONE_WIRE_DIR_OUT);	// drive output low
        one_wire_delay_us(10);
        one_wire_dev->set_pin_level(1);	                // drive output high
        interrupts();
        one_wire_delay_us(55);
    } else {
        noInterrupts();
        one_wire_dev->set_pin_level(0);
        one_wire_dev->set_pin_dir(ONE_WIRE_DIR_OUT);	// drive output low
        one_wire_delay_us(65);
        one_wire_dev->set_pin_level(1);	                // drive output high
        interrupts();
        one_wire_delay_us(5);
    }
	
	return 0;
}

/**
  * @brief   �����߶�1λ������
  * @param   one_wire_dev��one_wire�����豸����ָ��  readV����ȡ������
  * @retval  -1�������豸δ���� -2: �������ݵı���δ���� 0���ɹ�
 **/
int8_t one_wire_read_bit(one_wire_dev_t *one_wire_dev,uint8_t *readV)
{	
	if(one_wire_dev == NULL)
		return -1;
	
	if(readV == NULL)
		return -2;
	
    noInterrupts();
    one_wire_dev->set_pin_dir(ONE_WIRE_DIR_OUT);
    one_wire_dev->set_pin_level(0);
    one_wire_delay_us(3);
    one_wire_dev->set_pin_dir(ONE_WIRE_DIR_IN);	// let pin float, pull up will raise
    one_wire_delay_us(10);
    *readV = one_wire_dev->read_pin_level();
    interrupts();
    one_wire_delay_us(53);
    return 0;
}

/**
  * @brief   ������д��һ���ֽڡ�д������ʹ����Դ���������������ߡ��������Ҫ��д����������
  *      	 ���磨���磬���ü�����Դģʽ�� DS18S20������ 'power' ����Ϊ 1��������д�������
  *		     �������Ž���Ϊ��̬���Ա����ڶ�·��������������²�����������
  * @param   one_wire_dev��one_wire�����豸����ָ��  v��д�������  power��������Դ������
  * @retval  -1�������豸δ����  0���ɹ�
 **/
int8_t one_wire_write(one_wire_dev_t *one_wire_dev,uint8_t v, uint8_t power) 
{
    uint8_t bitMask;
	
	if(one_wire_dev == NULL)
		return -1;
	
    for (bitMask = 0x01; bitMask; bitMask <<= 1) 
	{
        one_wire_write_bit(one_wire_dev,(bitMask & v)?1:0);
    }
    if ( !power) {
        noInterrupts();
        one_wire_dev->set_pin_dir(ONE_WIRE_DIR_IN);
        one_wire_dev->set_pin_level(0);
        interrupts();
    }
	
	return 0;
}

/**
  * @brief   ������д�����ֽڡ�д������ʹ����Դ���������������ߡ��������Ҫ��д����������
  *      	 ���磨���磬���ü�����Դģʽ�� DS18S20������ 'power' ����Ϊ 1��������д�������
  *		     �������Ž���Ϊ��̬���Ա����ڶ�·��������������²�����������
  * @param   one_wire_dev��one_wire�����豸����ָ��  buf��д������ݻ��� count��д������ݸ��� power��������Դ������
  * @retval  -1�������豸δ����  0���ɹ�
 **/
int8_t one_wire_write_bytes(one_wire_dev_t *one_wire_dev,const uint8_t *buf, uint16_t count, bool power) 
{
    if(one_wire_dev == NULL)
		return -1;
	
	for (uint16_t i = 0 ; i < count ; i++)
        one_wire_write(one_wire_dev, buf[i], 0);
    if (!power) {
        noInterrupts();
        one_wire_dev->set_pin_dir(ONE_WIRE_DIR_IN);
        one_wire_dev->set_pin_level(0);
        interrupts();
    }
	return 0;
}

/**
  * @brief   �����߶�ȡһ���ֽڡ�
  * @param   one_wire_dev��one_wire�����豸����ָ��  read_data����ȡ������ 
  * @retval  -1�������豸δ���� -2��read_dataδ���� 0���ɹ�
 **/
int8_t one_wire_read(one_wire_dev_t *one_wire_dev,uint8_t *read_data) 
{
    uint8_t bitMask;
	uint8_t read_data_temp;
	
	if(one_wire_dev == NULL)
		return -1;
	
	if(read_data == NULL)
		return -2;
	
	*read_data = 0;
	
    for (bitMask = 0x01; bitMask; bitMask <<= 1) 
	{
		one_wire_read_bit(one_wire_dev,&read_data_temp);
        if (read_data_temp) 
			*read_data |= bitMask;
    }
    return 0;
}

/**
  * @brief   �����߶�ȡ����ֽڡ�
  * @param   one_wire_dev��one_wire�����豸����ָ��  buf����ȡ�����ݻ��� count����ȡ�����ݸ���
  * @retval  -1�������豸δ����  0���ɹ�
 **/
int8_t one_wire_read_bytes(one_wire_dev_t *one_wire_dev,uint8_t *buf, uint16_t count) 
{
	uint8_t read_data;
	
	if(one_wire_dev == NULL)
		return -1;
	
    for (uint16_t i = 0 ; i < count ; i++)
    {
		one_wire_read(one_wire_dev,&read_data);
		buf[i] = read_data;
	}
	return 0;
}

/**
  * @brief   ����ROMѡ����ָ���豸ͨ�š�ÿ���豸�����Լ���ROM����
  * @param   one_wire_dev��one_wire�����豸����ָ��  rom��ѡ����豸ROMֵ
  * @retval  -1�������豸δ����  0���ɹ�
 **/
int8_t one_wire_select(one_wire_dev_t *one_wire_dev,const uint8_t rom[8])
{
    uint8_t i;
	
	if(one_wire_dev == NULL)
		return -1;
	
    one_wire_write(one_wire_dev,0x55,0);           // Choose ROM

    for (i = 0; i < 8; i++) 
		one_wire_write(one_wire_dev,rom[i],0);
	
	return 0;
}


/**
  * @brief   ����ROMѡ���ʺϵ��豸��ͨ��
  * @param   one_wire_dev��one_wire�����豸����ָ��  rom��ѡ����豸ROMֵ
  * @retval  -1�������豸δ����  0���ɹ�
 **/
int8_t one_wire_skip(one_wire_dev_t *one_wire_dev)
{
	if(one_wire_dev == NULL)
		return -1;
    one_wire_write(one_wire_dev,0xCC,0);           // Skip ROM
	
	return 0;
}

/**
  * @brief   �����������룬��Դ���
  * @param   one_wire_dev��one_wire�����豸����ָ��   
  * @retval  -1�������豸δ����  0���ɹ�
 **/
int8_t one_wire_depower(one_wire_dev_t *one_wire_dev)
{
	if(one_wire_dev == NULL)
		return -1;
    noInterrupts();
    one_wire_dev->set_pin_dir(ONE_WIRE_DIR_IN);
    interrupts();
	
	return 0;
}

#if ONEWIRE_SEARCH

/**
  * @brief   ���õ����ߵ��������
  *          ����Ҫʹ�ô˺��������´ӿ�ͷ��ʼ���������ڵ�һ����������������������������Ҳ������ô����
  * @param   one_wire_dev��one_wire�����豸����ָ��   
  * @retval  -1�������豸δ����  0���ɹ�
 **/
int8_t one_wire_reset_search(one_wire_dev_t *one_wire_dev)
{
	if(one_wire_dev == NULL)
		return -1;
    // reset the search state
    one_wire_dev->LastDiscrepancy = 0;
    one_wire_dev->LastDeviceFlag = false;
    one_wire_dev->LastFamilyDiscrepancy = 0;
    for(int i = 7; ; i--) {
        one_wire_dev->ROM_NO[i] = 0;
        if ( i == 0) break;
    }
	return 0;
}

/**
  * @brief   �����������ã��Ա����´ε��� search(*newAddr) ʱ���������豸����Ϊ "family_code" ���豸�������ҵ�����
  * @param   one_wire_dev��one_wire�����豸����ָ��    family_code���豸����
  * @retval  -1�������豸δ����  0���ɹ�
 **/
int8_t one_wire_target_search(one_wire_dev_t *one_wire_dev,uint8_t family_code)
{
	if(one_wire_dev == NULL)
		return -1;
	
    // set the search state to find SearchFamily type devices
    one_wire_dev->ROM_NO[0] = family_code;
    for (uint8_t i = 1; i < 8; i++)
        one_wire_dev->ROM_NO[i] = 0;
    one_wire_dev->LastDiscrepancy = 64;
    one_wire_dev->LastFamilyDiscrepancy = 0;
    one_wire_dev->LastDeviceFlag = false;
	
	return 0;
}

/**
  * @brief   ִ��һ������������˺�������ֵΪ ��1�������ʾ��ö�ٵ���һ���豸��
  *          ����Դ� address �����л�ȡ�� ROM ��Ϣ������������豸��
  *          û�и����豸��������ö�ٹ����г������ش��󣬺��������� 0�������
  *          �����豸�����ַ�������Ƶ� newAddr �С�ʹ�� reset_search() �����¿�ʼ������
  * @param   one_wire_dev��one_wire�����豸����ָ��    family_code���豸����
  * @retval  -1�������豸δ����  0��δ�ҵ��豸����������  1: �ҵ��豸��ROM ����� ROM_NO ��������
 **/
int8_t one_wire_search(one_wire_dev_t *one_wire_dev, uint8_t *newAddr, bool search_mode /* = true */)
{
    uint8_t id_bit_number;
    uint8_t last_zero, rom_byte_number;
    bool  search_result;
    uint8_t id_bit, cmp_id_bit;

    unsigned char rom_byte_mask, search_direction;
	
	if(one_wire_dev == NULL)
		return -1;
	
    // initialize for search
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1;
    search_result = false;
	
    // if the last call was not the last one
    if (!one_wire_dev->LastDeviceFlag) {
        // 1-Wire reset
        if (!one_wire_reset(one_wire_dev)) {
            // reset the search
            one_wire_dev->LastDiscrepancy = 0;
            one_wire_dev->LastDeviceFlag = false;
            one_wire_dev->LastFamilyDiscrepancy = 0;
            return false;
        }

        // issue the search command
        if (search_mode == true) {
            one_wire_write(one_wire_dev,0xF0,0);   // NORMAL SEARCH
        } else {
            one_wire_write(one_wire_dev,0xEC,0);   // CONDITIONAL SEARCH
        }

        // loop to do the search
        do
        {
            // read a bit and its complement
            one_wire_read_bit(one_wire_dev,&id_bit);
            one_wire_read_bit(one_wire_dev,&cmp_id_bit);

            // check for no devices on 1-wire
            if ((id_bit == 1) && (cmp_id_bit == 1)) {
                break;
            } else {
                // all devices coupled have 0 or 1
                if (id_bit != cmp_id_bit) {
                    search_direction = id_bit;  // bit write value for search
                } else {
                    // if this discrepancy if before the Last Discrepancy
                    // on a previous next then pick the same as last time
                    if (id_bit_number < one_wire_dev->LastDiscrepancy) {
                        search_direction = ((one_wire_dev->ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
                    } else {
                        // if equal to last pick 1, if not then pick 0
                        search_direction = (id_bit_number == one_wire_dev->LastDiscrepancy);
                    }
                    // if 0 was picked then record its position in LastZero
                    if (search_direction == 0) {
                        last_zero = id_bit_number;

                        // check for Last discrepancy in family
                        if (last_zero < 9)
                            one_wire_dev->LastFamilyDiscrepancy = last_zero;
                    }
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction == 1)
                    one_wire_dev->ROM_NO[rom_byte_number] |= rom_byte_mask;
                else
                    one_wire_dev->ROM_NO[rom_byte_number] &= ~rom_byte_mask;

                // serial number search direction write bit
                one_wire_write_bit(one_wire_dev,search_direction);

                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0) {
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        }
        while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

        // if the search was successful then
        if (!(id_bit_number < 65)) {
            // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
            one_wire_dev->LastDiscrepancy = last_zero;

            // check for last device
            if (one_wire_dev->LastDiscrepancy == 0) {
                one_wire_dev->LastDeviceFlag = true;
            }
            search_result = true;
        }
    }

    // if no device found then reset counters so next 'search' will be like a first
    if (!search_result || ! one_wire_dev->ROM_NO[0]) {
        one_wire_dev->LastDiscrepancy = 0;
        one_wire_dev->LastDeviceFlag = false;
        one_wire_dev->LastFamilyDiscrepancy = 0;
        search_result = false;
    } else {
        for (int i = 0; i < 8; i++) 
			newAddr[i] = one_wire_dev->ROM_NO[i];
    }
    return search_result;
}

#endif

#if ONEWIRE_CRC
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

#if ONEWIRE_CRC8_TABLE
// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t dscrc2x16_table[] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
    0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};
#define pgm_read_byte(addr) (*(const uint8_t *)(addr))
// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Use tiny 2x16 entry CRC table)
uint8_t crc8(const uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0;

    while (len--) {
        crc = *addr++ ^ crc;  // just re-using crc as intermediate
        crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
              pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
    }

    return crc;
}
#else
//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but a little smaller, than the lookup table.
//
uint8_t OneWire::crc8(const uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0;

    while (len--) {
#if defined(__AVR__)
        crc = _crc_ibutton_update(crc, *addr++);
#else
        uint8_t inbyte = *addr++;
        for (uint8_t i = 8; i; i--) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
#endif
    }
    return crc;
}
#endif

#if ONEWIRE_CRC16
bool check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc)
{
    crc = ~crc16(input, len, crc);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}

uint16_t crc16(const uint8_t* input, uint16_t len, uint16_t crc)
{
    static const uint8_t oddparity[16] =
    { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    for (uint16_t i = 0 ; i < len ; i++) {
        // Even though we're just copying a byte from the input,
        // we'll be doing 16-bit computation with it.
        uint16_t cdata = input[i];
        cdata = (cdata ^ crc) & 0xff;
        crc >>= 8;

        if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
            crc ^= 0xC001;

        cdata <<= 6;
        crc ^= cdata;
        cdata <<= 1;
        crc ^= cdata;
    }
    return crc;
}
#endif

#endif

