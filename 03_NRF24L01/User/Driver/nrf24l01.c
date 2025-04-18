/**
  ******************************************************************************
  * @file    nrf24l01.c
  * @author  ����Ǳ��(snqx-lgh)
  * @version V
  * @date    2025-4-8
  * @brief   ������ԭ�ӿ�Դ�Ĵ�������ϣ�������û��ӿڵĲ��֣�ֻ��Ҫ�޸��û��ӿ�
  * 		 ���ݼ���
  * @github  https://github.com/snqx-lqh
  * @gitee   https://gitee.com/snqx-lqh
  * @wiki    https://snqx-lqh.github.io/wiki
  ******************************************************************************
  * @attention
  * 
  *
  * <h2><center>&copy; Copyright {Year} LQH,China</center></h2>
  ******************************************************************************
  */
#include "nrf24l01.h"

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]= {0x11,0x22,0x33,0x44,0x55}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]= {0x11,0x22,0x33,0x44,0x55}; //���յ�ַ

/********************************************/
//����SPI����ʽ���������ű仯�Ĵ���
#include "main.h"
#include "spi.h"
#include "bsp_soft_spi.h"

//�Ƿ�ʹ�����SPI�����ʹ�����SPI��Ҫʹ������
#define USE_SOFTWARE_SPI 0

#if USE_SOFTWARE_SPI
static void set_sck_level(uint8_t level)
{
	if(0 == level){
		HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port,SPI1_SCK_Pin,GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port,SPI1_SCK_Pin,GPIO_PIN_SET);
	}
}

static void set_mosi_level(uint8_t level)
{
	if(0 == level){
		HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port,SPI1_MOSI_Pin,GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port,SPI1_MOSI_Pin,GPIO_PIN_SET);
	}
}

static uint8_t get_miso_level( )
{
	uint8_t level;
	level = HAL_GPIO_ReadPin(SPI1_MISO_GPIO_Port,SPI1_MISO_Pin);
	return level; 
}
#endif 

static void set_ce_level(uint8_t level)
{
	if(0 == level){
		HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET);
	}
}

static void set_csn_level(uint8_t level)
{
	if(0 == level){
		HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);
	}
}

static uint8_t get_irq_level( )
{
	uint8_t level;
	level = HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port,NRF_IRQ_Pin);
	return level; 
}

static void nrf_spi_init()
{
	//��ʼ�����ŵȣ�HAL��ͼ�λ�������Լ���ʼ��
	
	//��ʼ��SPI�ȣ� HAL��ͼ�λ�������Լ���ʼ��
	
}

#if USE_SOFTWARE_SPI
soft_spi_t soft_spi = {
	.spi_init           = nrf_spi_init,
	.set_spi_sck_level  = set_sck_level,
	.set_spi_mosi_level = set_mosi_level,
	.spi_miso_read      = get_miso_level,
    .spi_mode=  0,
};
#endif

/**
  * @brief   NRF��SPI�շ����� 
  * @param    
  * @retval
 **/
uint8_t nrf_read_write_byte(uint8_t TxData)
{
	uint8_t RxData;
	#if USE_SOFTWARE_SPI
	RxData = soft_spi_read_write_byte(&soft_spi,TxData);
	#else
	HAL_SPI_TransmitReceive(&hspi1,&TxData,&RxData,1, 1000);  
	#endif
	return RxData;
}


/********************************************/

/**
  * @brief   NRF24L01�ĳ�ʼ�����������ź�SPI�ĳ�ʼ��
  * @param    
  * @retval  0���ɹ�;1��ʧ�� 
 **/
uint8_t NRF24L01_Init(void)
{
	int16_t check_nrf = 0;    //NRF��ʼ������
	// ��ʼ��SPI�������
	nrf_spi_init();
	
    set_ce_level(0) ;  //ʹ��24L01
    set_csn_level(1); //SPIƬѡȡ��
	
	// ���NRF24L01�Ƿ��ܳɹ���ȡ
	check_nrf = NRF24L01_Check();
	// ʹ��Ϊ����ģʽ
	NRF24L01_RX_Mode();
	
	return check_nrf;
}
/**
  * @brief     NRFʹ��SPIͨ��д�Ĵ���
  * @param	   reg:ָ���Ĵ�����ַ
  * @param     value:д���ֵ
  * @retval    �Ƿ�ɹ���״ֵ̬
 **/
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
    uint8_t status;
    set_csn_level(0);                 //ʹ��SPI����
    status =nrf_read_write_byte(reg); //���ͼĴ�����
    nrf_read_write_byte(value);       //д��Ĵ�����ֵ
    set_csn_level(1);                 //��ֹSPI����
    return status;                    
}

/**
  * @brief     NRFʹ��SPIͨ�Ŷ��Ĵ���ֵ
  * @param     reg:Ҫ���ļĴ���
  * @retval    �Ƿ�ɹ���״ֵ̬ 
 **/
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    set_csn_level(0);          //ʹ��SPI����
    nrf_read_write_byte(reg);   //���ͼĴ�����
    reg_val=nrf_read_write_byte(0XFF);//��ȡ�Ĵ�������
    set_csn_level(1);          //��ֹSPI����
    return reg_val;           //����״ֵ̬
}

/**
  * @brief     NRFʹ��SPIͨ����ָ��λ�ö���ָ�����ȵ�����
  * @param     reg:Ҫ���ļĴ���
  * @param     *pBuf:����ָ��
  * @param     len:���ݳ���
  * @retval    �Ƿ�ɹ���״ֵ̬ 
 **/
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
    uint8_t status,uint8_t_ctr;
    set_csn_level(0);           //ʹ��SPI����
    status=nrf_read_write_byte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
    for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
        pBuf[uint8_t_ctr]=nrf_read_write_byte(0XFF);//��������
    set_csn_level(1);       //�ر�SPI����
    return status;        //���ض�����״ֵ̬
}

/**
  * @brief     NRFʹ��SPIͨ����ָ��λ��дָ�����ȵ�����
  * @param     reg:Ҫ���ļĴ���
  * @param     *pBuf:����ָ��
  * @param     len:���ݳ���
  * @retval    �Ƿ�ɹ���״ֵ̬ 
 **/
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status,uint8_t_ctr;
    set_csn_level(0);          //ʹ��SPI����
    status = nrf_read_write_byte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
    for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
        nrf_read_write_byte(*pBuf++); //д������
    set_csn_level(1);       //�ر�SPI����
    return status;          //���ض�����״ֵ̬
}
/**
  * @brief     �ú�����ʼ��NRF24L01��RXģʽ,����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
  * 		   ��CE��ߺ�,������RXģʽ,�����Խ���������
  * @param     void
  * @retval    void 
 **/
void NRF24L01_RX_Mode(void)
{
    set_ce_level(0);
    NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);
    //дRX�ڵ��ַ
    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��
    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);           //����RFͨ��Ƶ��
    NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
    //��32���ֽ�
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);      //����TX�������,0db����,2Mbps,���������濪��
    NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);       //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ
    //��CONFIG(00)д��0000 1111;PRIM_RX=1 ����ģʽ; PWR_UP=1 �ϵ�; CRCO=1 16λCRCУ��; EN_CRC =1 CRCʹ��
    set_ce_level(1); //CEΪ��,�������ģʽ
}
/**
  * @brief     �ú�����ʼ��NRF24L01��TXģʽ,����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,
  * 		   ѡ��RFƵ��,�����ʺ�LNA HCURR
  * 		   PWR_UP,CRCʹ��,CEΪ�ߴ���10us,����������.
  * @param     void
  * @retval    void 
 **/
void NRF24L01_TX_Mode(void)
{
    set_ce_level(0);
    NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);
    //дTX�ڵ��ַ
    NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);
    //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK
    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��
    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ
    NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��
    NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	
    //��CONFIG(00)д��0000 1111;PRIM_RX=0 ����ģʽ; PWR_UP=1 �ϵ�; CRCO=1 16λCRCУ��; EN_CRC   =1 CRCʹ��
    set_ce_level(1);//CEΪ��,10us����������
}

/**
  * @brief     ����NRF24L01����һ������
  * @param     txbuf:�����������׵�ַ
  * @retval    �������״��
 **/
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    uint8_t sta;
    set_ce_level(0);
    NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
    set_ce_level(1);//��������
    while(get_irq_level()!=0);//�ȴ��������
    sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
    NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
    if(sta&MAX_TX)//�ﵽ����ط�����
    {
        NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ���
        return MAX_TX;
    }
    if(sta&TX_OK)//�������
    {
        return TX_OK;
    }
    return 0xff;//����ԭ����ʧ��
}

/**
  * @brief     ����NRF24L01����һ������
  * @param     rxbuf:�����������׵�ַ
  * @retval    0��������ɣ��������������
 **/
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t sta;
    sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
    NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
    if(sta&RX_OK)//���յ�����
    {
        NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
        NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ���
        return 0;
    }
    return 1;//û�յ��κ�����
}

/**
  * @brief     ���24L01�Ƿ����
  * @param     void
  * @retval    0���ɹ�;1��ʧ��
 **/
uint8_t NRF24L01_Check_Buff[5]= {0XAA,0XAA,0XAA,0XAA,0XAA}; //д��5��0XAA�ֽ�
uint8_t NRF24L01_Check(void)
{
    uint8_t i;
    NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,NRF24L01_Check_Buff,5);//д��5���ֽڵĵ�ַ.
    NRF24L01_Read_Buf(TX_ADDR,NRF24L01_Check_Buff,5); //����д��ĵ�ַ
    for(i=0; i<5; i++)
        if(NRF24L01_Check_Buff[i]!=0XAA)
            break;
    if(i!=5)return 1;//���24L01����
    return 0;        //��⵽24L01
}
