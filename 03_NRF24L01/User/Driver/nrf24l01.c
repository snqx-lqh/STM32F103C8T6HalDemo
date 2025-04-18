/**
  ******************************************************************************
  * @file    nrf24l01.c
  * @author  少年潜行(snqx-lgh)
  * @version V
  * @date    2025-4-8
  * @brief   在正点原子开源的代码基础上，添加了用户接口的部分，只需要修改用户接口
  * 		 内容即可
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

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]= {0x11,0x22,0x33,0x44,0x55}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]= {0x11,0x22,0x33,0x44,0x55}; //接收地址

/********************************************/
//设置SPI处理方式，设置引脚变化的处理
#include "main.h"
#include "spi.h"
#include "bsp_soft_spi.h"

//是否使用软件SPI，如果使用软件SPI需要使用引脚
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
	//初始化引脚等，HAL库图形化界面会自己初始化
	
	//初始化SPI等， HAL库图形化界面会自己初始化
	
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
  * @brief   NRF的SPI收发数据 
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
  * @brief   NRF24L01的初始化，包含引脚和SPI的初始化
  * @param    
  * @retval  0，成功;1，失败 
 **/
uint8_t NRF24L01_Init(void)
{
	int16_t check_nrf = 0;    //NRF初始化检验
	// 初始化SPI相关配置
	nrf_spi_init();
	
    set_ce_level(0) ;  //使能24L01
    set_csn_level(1); //SPI片选取消
	
	// 检查NRF24L01是否能成功读取
	check_nrf = NRF24L01_Check();
	// 使能为接收模式
	NRF24L01_RX_Mode();
	
	return check_nrf;
}
/**
  * @brief     NRF使用SPI通信写寄存器
  * @param	   reg:指定寄存器地址
  * @param     value:写入的值
  * @retval    是否成功的状态值
 **/
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
    uint8_t status;
    set_csn_level(0);                 //使能SPI传输
    status =nrf_read_write_byte(reg); //发送寄存器号
    nrf_read_write_byte(value);       //写入寄存器的值
    set_csn_level(1);                 //禁止SPI传输
    return status;                    
}

/**
  * @brief     NRF使用SPI通信读寄存器值
  * @param     reg:要读的寄存器
  * @retval    是否成功的状态值 
 **/
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    set_csn_level(0);          //使能SPI传输
    nrf_read_write_byte(reg);   //发送寄存器号
    reg_val=nrf_read_write_byte(0XFF);//读取寄存器内容
    set_csn_level(1);          //禁止SPI传输
    return reg_val;           //返回状态值
}

/**
  * @brief     NRF使用SPI通信在指定位置读出指定长度的数据
  * @param     reg:要读的寄存器
  * @param     *pBuf:数据指针
  * @param     len:数据长度
  * @retval    是否成功的状态值 
 **/
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
    uint8_t status,uint8_t_ctr;
    set_csn_level(0);           //使能SPI传输
    status=nrf_read_write_byte(reg);//发送寄存器值(位置),并读取状态值
    for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
        pBuf[uint8_t_ctr]=nrf_read_write_byte(0XFF);//读出数据
    set_csn_level(1);       //关闭SPI传输
    return status;        //返回读到的状态值
}

/**
  * @brief     NRF使用SPI通信在指定位置写指定长度的数据
  * @param     reg:要读的寄存器
  * @param     *pBuf:数据指针
  * @param     len:数据长度
  * @retval    是否成功的状态值 
 **/
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status,uint8_t_ctr;
    set_csn_level(0);          //使能SPI传输
    status = nrf_read_write_byte(reg);//发送寄存器值(位置),并读取状态值
    for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
        nrf_read_write_byte(*pBuf++); //写入数据
    set_csn_level(1);       //关闭SPI传输
    return status;          //返回读到的状态值
}
/**
  * @brief     该函数初始化NRF24L01到RX模式,设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
  * 		   当CE变高后,即进入RX模式,并可以接收数据了
  * @param     void
  * @retval    void 
 **/
void NRF24L01_RX_Mode(void)
{
    set_ce_level(0);
    NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);
    //写RX节点地址
    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答
    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);           //设置RF通信频率
    NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
    //即32个字节
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);      //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
    NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);       //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式
    //向CONFIG(00)写入0000 1111;PRIM_RX=1 接收模式; PWR_UP=1 上电; CRCO=1 16位CRC校验; EN_CRC =1 CRC使能
    set_ce_level(1); //CE为高,进入接收模式
}
/**
  * @brief     该函数初始化NRF24L01到TX模式,设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,
  * 		   选择RF频道,波特率和LNA HCURR
  * 		   PWR_UP,CRC使能,CE为高大于10us,则启动发送.
  * @param     void
  * @retval    void 
 **/
void NRF24L01_TX_Mode(void)
{
    set_ce_level(0);
    NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);
    //写TX节点地址
    NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);
    //设置TX节点地址,主要为了使能ACK
    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答
    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址
    NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //设置RF通道为40
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
    NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	
    //向CONFIG(00)写入0000 1111;PRIM_RX=0 发射模式; PWR_UP=1 上电; CRCO=1 16位CRC校验; EN_CRC   =1 CRC使能
    set_ce_level(1);//CE为高,10us后启动发送
}

/**
  * @brief     启动NRF24L01发送一次数据
  * @param     txbuf:待发送数据首地址
  * @retval    发送完成状况
 **/
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    uint8_t sta;
    set_ce_level(0);
    NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
    set_ce_level(1);//启动发送
    while(get_irq_level()!=0);//等待发送完成
    sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
    NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
    if(sta&MAX_TX)//达到最大重发次数
    {
        NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器
        return MAX_TX;
    }
    if(sta&TX_OK)//发送完成
    {
        return TX_OK;
    }
    return 0xff;//其他原因发送失败
}

/**
  * @brief     启动NRF24L01接收一次数据
  * @param     rxbuf:待接收数据首地址
  * @retval    0，接收完成；其他，错误代码
 **/
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t sta;
    sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
    NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
    if(sta&RX_OK)//接收到数据
    {
        NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
        NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器
        return 0;
    }
    return 1;//没收到任何数据
}

/**
  * @brief     检测24L01是否存在
  * @param     void
  * @retval    0，成功;1，失败
 **/
uint8_t NRF24L01_Check_Buff[5]= {0XAA,0XAA,0XAA,0XAA,0XAA}; //写入5个0XAA字节
uint8_t NRF24L01_Check(void)
{
    uint8_t i;
    NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,NRF24L01_Check_Buff,5);//写入5个字节的地址.
    NRF24L01_Read_Buf(TX_ADDR,NRF24L01_Check_Buff,5); //读出写入的地址
    for(i=0; i<5; i++)
        if(NRF24L01_Check_Buff[i]!=0XAA)
            break;
    if(i!=5)return 1;//检测24L01错误
    return 0;        //检测到24L01
}
