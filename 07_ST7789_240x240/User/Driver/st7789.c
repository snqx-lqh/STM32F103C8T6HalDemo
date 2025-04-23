#include "st7789.h"
#include "font.h"

/**************** �û���������  Start *********************/
// �ⲿ�ִ����Լ�д��ʱ����Է��ļ�

#define LCD_DELAY_XMS(XMS)  HAL_Delay(XMS)

#define LCD_USE_SOFT_SPI 0

/**
  * @brief   ��ʼ��LCD������ţ��Լ���ʼ��SPI�����ʹ�������spi��Ҫ��ʼ�������ص�����
  * @param   : [����/��] 
  * @retval
 **/
void lcd_spi_init()
{
	// ʹ�õ�HAL���Ѿ��Լ���ʼ����
}

/**
  * @brief   ����RES���ŵ�ƽ
  * @param   level : ��ƽ 0�� 1��
  * @retval
 **/
void lcd_res_set_level(uint8_t level)
{
	if(0 == level)
	{
		HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin,GPIO_PIN_RESET);
	}else if(1 == level)
	{
		HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin,GPIO_PIN_SET);
	}
}

/**
  * @brief   ����DC���ŵ�ƽ
  * @param   level : ��ƽ 0�� 1��
  * @retval
 **/
void lcd_dc_set_level(uint8_t level)
{
	if(0 == level)
	{
		HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_RESET);
	}else if(1 == level)
	{
		HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_SET);
	}
}

/**
  * @brief   ����CS���ŵ�ƽ
  * @param   level : ��ƽ 0�� 1��
  * @retval
 **/
void lcd_cs_set_level(uint8_t level)
{
	if(0 == level)
	{
		 
	}else if(1 == level)
	{
		 
	}
}

/**
  * @brief   ����BLK���ŵ�ƽ
  * @param   level : ��ƽ 0�� 1��
  * @retval
 **/
void lcd_blk_set_level(uint8_t level)
{
	if(0 == level)
	{
		HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin,GPIO_PIN_RESET);
	}else if(1 == level)
	{
		HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin,GPIO_PIN_SET);
	}
}

#if LCD_USE_SOFT_SPI
#include "bsp_soft_spi.h"

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

//�������SPI����
soft_spi_t soft_spi = {
	.spi_init           = lcd_spi_init,
	.set_spi_sck_level  = set_sck_level,
	.set_spi_mosi_level = set_mosi_level,
	.spi_miso_read      = get_miso_level,
    .spi_mode=  2,
};
#else

#include "spi.h"

#endif
/**
  * @brief   lcd��SPI�շ����� 
  * @param    
  * @retval
 **/
uint8_t lcd_read_write_byte(uint8_t TxData)
{
	uint8_t RxData;
	#if LCD_USE_SOFT_SPI
		RxData = soft_spi_read_write_byte(&soft_spi,TxData);	
	#else
		HAL_SPI_TransmitReceive(&hspi1,&TxData,&RxData,1, 100);  
	#endif
	return RxData;
}

/**************** �û���������  end ******************/

/**
  * @brief   LCD��������д�뺯��
  * @param   dat  Ҫд��Ĵ�������
  * @retval
 **/
void LCD_Writ_Bus(uint8_t dat) 
{	
	lcd_cs_set_level(0);
	
	lcd_read_write_byte(dat);
	
  	lcd_cs_set_level(1);
}

/**
  * @brief   LCDд������
  * @param   dat д�������
  * @retval
 **/
void LCD_WR_DATA8(uint8_t dat)
{
    lcd_dc_set_level(1);;//д����
	LCD_Writ_Bus(dat);
}

/**
  * @brief   LCDд������
  * @param   dat д�������
  * @retval
 **/
void LCD_WR_DATA(uint16_t dat)
{
    lcd_dc_set_level(1);//д����
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}
/**
  * @brief   LCDд������
  * @param   dat д�������
  * @retval
 **/
void LCD_WR_REG(uint8_t dat)
{
	lcd_dc_set_level(0);//д����
	LCD_Writ_Bus(dat);
	lcd_dc_set_level(1);//д����
}

/**
  * @brief   ������ʼ�ͽ�����ַ
  * @param   x1,x2 �����е���ʼ�ͽ�����ַ
  *          y1,y2 �����е���ʼ�ͽ�����ַ
  * @retval
 **/
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//�е�ַ����
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//�е�ַ����
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//������д
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//�е�ַ����
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//�е�ַ����
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//������д
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//�е�ַ����
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//�е�ַ����
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//������д
	}
	else
	{
		LCD_WR_REG(0x2a);//�е�ַ����
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//�е�ַ����
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//������д
	}
}

/**
  * @brief   LCD�ĳ�ʼ��
  * @param   
  * @retval
 **/
void LCD_Init(void)
{
	lcd_spi_init();//��ʼ��GPIO
	lcd_blk_set_level(0);//�رձ���

	lcd_res_set_level(0);//��λ
	LCD_DELAY_XMS(50);
	lcd_res_set_level(1);
	LCD_DELAY_XMS(50);
	
    lcd_blk_set_level(1);//�򿪱���

	LCD_WR_REG(0x11); 
	LCD_DELAY_XMS(10); 
	LCD_WR_REG(0x36); 
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x33); 

	LCD_WR_REG(0xB7); 
	LCD_WR_DATA8(0x35);  

	LCD_WR_REG(0xBB);
	LCD_WR_DATA8(0x35);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x2C);

	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x13);   

	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x20);  

	LCD_WR_REG(0xC6); 
	LCD_WR_DATA8(0x0F);    

	LCD_WR_REG(0xD0); 
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1);

	LCD_WR_REG(0xD6); 
	LCD_WR_DATA8(0xA1);

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xF0);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x29);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x3E);
	LCD_WR_DATA8(0x38);
	LCD_WR_DATA8(0x12);
	LCD_WR_DATA8(0x12);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x30);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0xF0);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0B);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x3E);
	LCD_WR_DATA8(0x36);
	LCD_WR_DATA8(0x14);
	LCD_WR_DATA8(0x14);
	LCD_WR_DATA8(0x29);
	LCD_WR_DATA8(0x32);

	LCD_WR_REG(0x21); 
  
  	LCD_WR_REG(0x11);
  	LCD_DELAY_XMS(120);	
	LCD_WR_REG(0x29);

} 

/**
  * @brief   ��ָ�����������ɫ
  * @param   xsta,ysta   ��ʼ����
  *          xend,yend   ��ֹ����
  *          color       Ҫ������ɫ
  * @retval
 **/
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{          
	uint16_t i,j; 
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//������ʾ��Χ
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}

/**
  * @brief   ��ָ��λ�û���
  * @param   x,y ��������
  *          color �����ɫ
  * @retval
 **/
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Address_Set(x,y,x,y);//���ù��λ�� 
	LCD_WR_DATA(color);
} 

/**
  * @brief   ����
  * @param   x1,y1   ��ʼ����
  *          x2,y2   ��ֹ����
  *          color   �ߵ���ɫ
  * @retval
 **/
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //������������ 
	delta_y=y2-y1;
	uRow=x1;//�����������
	uCol=y1;
	if(delta_x>0)incx=1; //���õ������� 
	else if (delta_x==0)incx=0;//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//ˮƽ�� 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//����
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

/**
  * @brief   ������
  * @param   x1,y1   ��ʼ����
  *          x2,y2   ��ֹ����
  *          color   �ߵ���ɫ
  * @retval
 **/
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}

/**
  * @brief   ��Բ
  * @param   x0,y0   Բ������
  *          r       �뾶
  *          color   Բ����ɫ
  * @retval
 **/
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//�ж�Ҫ���ĵ��Ƿ��Զ
		{
			b--;
		}
	}
}
/**
  * @brief   ��ʾ���ִ�
  * @param   x,y ��ʾ����
  *          *s  Ҫ��ʾ�ĺ��ִ�
  *          fc �ֵ���ɫ
  *          bc �ֵı���ɫ
  *          sizey �ֺ� ��ѡ 16 24 32
  *          mode:  0�ǵ���ģʽ  1����ģʽ
  * @retval
 **/
 void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
 {
 	while(*s!=0)
 	{
 		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
 		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
 		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
 		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
 		else return;
 		s+=2;
 		x+=sizey;
 	}
 }

/**
  * @brief   ��ʾ����12x12����
  * @param   x,y ��ʾ����
  *          *s  Ҫ��ʾ�ĺ��ִ�
  *          fc �ֵ���ɫ
  *          bc �ֵı���ɫ
  *          sizey �ֺ� ��ѡ 16 24 32
  *          mode:  0�ǵ���ģʽ  1����ģʽ
  * @retval
 **/
 void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
 {
 	uint8_t i,j,m=0;
 	uint16_t k;
 	uint16_t HZnum;//������Ŀ
 	uint16_t TypefaceNum;//һ���ַ���ռ�ֽڴ�С
 	uint16_t x0=x;
 	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
 	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//ͳ�ƺ�����Ŀ
 	for(k=0;k<HZnum;k++) 
 	{
 		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
 		{ 	
 			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
 			for(i=0;i<TypefaceNum;i++)
 			{
 				for(j=0;j<8;j++)
 				{	
 					if(!mode)//�ǵ��ӷ�ʽ
 					{
 						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
 						else LCD_WR_DATA(bc);
 						m++;
 						if(m%sizey==0)
 						{
 							m=0;
 							break;
 						}
 					}
 					else//���ӷ�ʽ
 					{
 						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
 						x++;
 						if((x-x0)==sizey)
 						{
 							x=x0;
 							y++;
 							break;
 						}
 					}
 				}
 			}
 		}				  	
 		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
 	}
 } 

/**
  * @brief   ��ʾ����16x16����
  * @param   x,y ��ʾ����
  *          *s  Ҫ��ʾ�ĺ��ִ�
  *          fc �ֵ���ɫ
  *          bc �ֵı���ɫ
  *          sizey �ֺ� ��ѡ 16 24 32
  *          mode:  0�ǵ���ģʽ  1����ģʽ
  * @retval
 **/
 void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
 {
 	uint8_t i,j,m=0;
 	uint16_t k;
 	uint16_t HZnum;//������Ŀ
 	uint16_t TypefaceNum;//һ���ַ���ռ�ֽڴ�С
 	uint16_t x0=x;
   TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
 	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//ͳ�ƺ�����Ŀ
 	for(k=0;k<HZnum;k++) 
 	{
 		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
 		{ 	
 			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
 			for(i=0;i<TypefaceNum;i++)
 			{
 				for(j=0;j<8;j++)
 				{	
 					if(!mode)//�ǵ��ӷ�ʽ
 					{
 						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
 						else LCD_WR_DATA(bc);
 						m++;
 						if(m%sizey==0)
 						{
 							m=0;
 							break;
 						}
 					}
 					else//���ӷ�ʽ
 					{
 						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
 						x++;
 						if((x-x0)==sizey)
 						{
 							x=x0;
 							y++;
 							break;
 						}
 					}
 				}
 			}
 		}				  	
 		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
 	}
 } 

/**
  * @brief   ��ʾ����24x24����
  * @param   x,y ��ʾ����
  *          *s  Ҫ��ʾ�ĺ��ִ�
  *          fc �ֵ���ɫ
  *          bc �ֵı���ɫ
  *          sizey �ֺ� ��ѡ 16 24 32
  *          mode:  0�ǵ���ģʽ  1����ģʽ
  * @retval
 **/
 void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
 {
 	uint8_t i,j,m=0;
 	uint16_t k;
 	uint16_t HZnum;//������Ŀ
 	uint16_t TypefaceNum;//һ���ַ���ռ�ֽڴ�С
 	uint16_t x0=x;
 	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
 	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//ͳ�ƺ�����Ŀ
 	for(k=0;k<HZnum;k++) 
 	{
 		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
 		{ 	
 			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
 			for(i=0;i<TypefaceNum;i++)
 			{
 				for(j=0;j<8;j++)
 				{	
 					if(!mode)//�ǵ��ӷ�ʽ
 					{
 						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
 						else LCD_WR_DATA(bc);
 						m++;
 						if(m%sizey==0)
 						{
 							m=0;
 							break;
 						}
 					}
 					else//���ӷ�ʽ
 					{
 						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
 						x++;
 						if((x-x0)==sizey)
 						{
 							x=x0;
 							y++;
 							break;
 						}
 					}
 				}
 			}
 		}				  	
 		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
 	}
 } 

/**
  * @brief   ��ʾ����32x32����
  * @param   x,y ��ʾ����
  *          *s  Ҫ��ʾ�ĺ��ִ�
  *          fc �ֵ���ɫ
  *          bc �ֵı���ɫ
  *          sizey �ֺ� ��ѡ 16 24 32
  *          mode:  0�ǵ���ģʽ  1����ģʽ
  * @retval
 **/
 void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
 {
 	uint8_t i,j,m=0;
 	uint16_t k;
 	uint16_t HZnum;//������Ŀ
 	uint16_t TypefaceNum;//һ���ַ���ռ�ֽڴ�С
 	uint16_t x0=x;
 	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
 	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//ͳ�ƺ�����Ŀ
 	for(k=0;k<HZnum;k++) 
 	{
 		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
 		{ 	
 			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
 			for(i=0;i<TypefaceNum;i++)
 			{
 				for(j=0;j<8;j++)
 				{	
 					if(!mode)//�ǵ��ӷ�ʽ
 					{
 						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
 						else LCD_WR_DATA(bc);
 						m++;
 						if(m%sizey==0)
 						{
 							m=0;
 							break;
 						}
 					}
 					else//���ӷ�ʽ
 					{
 						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
 						x++;
 						if((x-x0)==sizey)
 						{
 							x=x0;
 							y++;
 							break;
 						}
 					}
 				}
 			}
 		}				  	
 		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
 	}
 }

/**
  * @brief   ��ʾ�����ַ�
  * @param   x,y ��ʾ����
  *          num Ҫ��ʾ���ַ�
  *          fc �ֵ���ɫ
  *          bc �ֵı���ɫ
  *          sizey �ֺ� ��ѡ 16 24 32
  *          mode:  0�ǵ���ģʽ  1����ģʽ
  * @retval
 **/
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	uint16_t x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //�õ�ƫ�ƺ��ֵ
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //���ù��λ�� 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //����6x12����
		else if(sizey==16)temp=ascii_1608[num][i];		 //����8x16����
		else if(sizey==24)temp=ascii_2412[num][i];		 //����12x24����
		else if(sizey==32)temp=ascii_3216[num][i];		 //����16x32����
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//�ǵ���ģʽ
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//����ģʽ
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//��һ����
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}

/**
  * @brief   ��ʾ�ַ���
  * @param   x,y ��ʾ����
  *          *p Ҫ��ʾ���ַ���
  *          fc �ֵ���ɫ
  *          bc �ֵı���ɫ
  *          sizey �ֺ� ��ѡ 16 24 32
  *          mode:  0�ǵ���ģʽ  1����ģʽ
  * @retval
 **/
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}

/**
  * @brief   ����
  * @param   m������nָ��
  * @retval
 **/
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}

/**
  * @brief   ��ʾ��������
  * @param   x,y ��ʾ����
  *          num Ҫ��ʾ��������
  *          len Ҫ��ʾ��λ��
  *          fc �ֵ���ɫ
  *          bc �ֵı���ɫ
  *          sizey �ֺ�
  * @retval
 **/
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	uint8_t sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 

/**
  * @brief   ��ʾ��λС������
  * @param   x,y ��ʾ����
  *          num Ҫ��ʾС������
  *          len Ҫ��ʾ��λ��
  *          fc �ֵ���ɫ
  *          bc �ֵı���ɫ
  *          sizey �ֺ�
  * @retval
 **/
void LCD_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp,sizex;
	uint16_t num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}

/**
  * @brief   ��ʾͼƬ
  * @param   x,y�������
  *          length ͼƬ����
  *          width  ͼƬ���
  *          pic[]  ͼƬ����   
  * @retval
 **/
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[])
{
	uint16_t i,j;
	uint32_t k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}
