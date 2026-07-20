#include "lcd.h"
#include "lcd_init.h"
#include "lcdfont.h"
#include "Delay.h"
#include "ti_msp_dl_config.h"
#include <math.h> 
#include <stdio.h>  


/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
								color       要填充的颜色
      返回值：  无
******************************************************************************/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j; 
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}

/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
	LCD_Address_Set(x,y,x,y);//设置光标位置 
	LCD_WR_DATA(color);
} 


/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向 
	else if (delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//画点
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


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
      返回值：  无
******************************************************************************/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
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
		if((a*a+b*b)>(r*r))//判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
      函数说明：显示汉字串
      入口数据：x,y显示坐标
                *s 要显示的汉字串
                fc 字的颜色
                bc 字的背景色
                sizey 字号 可选 16 24 32
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	while(*s!=0)
	{
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;
		s+=3;
		x+=sizey;
	}
}

/******************************************************************************
      函数说明：显示单个12x12汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese12x12(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
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
					else//叠加方式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
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
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

/******************************************************************************
      函数说明：显示单个16x16汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese16x16(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
  TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
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
					else//叠加方式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
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
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 


/******************************************************************************
      函数说明：显示单个24x24汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese24x24(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
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
					else//叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
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
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

/******************************************************************************
      函数说明：显示单个32x32汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese32x32(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
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
					else//叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
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
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}


/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 temp,sizex,t,m=0;
	u16 i,TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //得到偏移后的值
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //设置光标位置 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //调用6x12字体
		else if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
		else if(sizey==24)temp=ascii_2412[num][i];		 //调用12x24字体
		else if(sizey==32)temp=ascii_3216[num][i];		 //调用16x32字体
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//非叠加模式
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
			else//叠加模式
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
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


/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;
	return result;
}


/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
//void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey)
//{         	
//	u8 t,temp;
//	u8 enshow=0;
//	u8 sizex=sizey/2;
//	for(t=0;t<len;t++)
//	{
//		temp=(num/mypow(10,len-t-1))%10;
//		if(enshow==0&&t<(len-1))
//		{
//			if(temp==0)
//			{
//				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
//				continue;
//			}else enshow=1; 
//		 	 
//		}
//	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
//	}
//} 

#define MAX_INT_CACHE_SLOTS 10
#define MAX_INT_CHARS 10  // 整数最多10位(含负号)

void LCD_ShowIntNum(u16 x, u16 y, int num, u16 fc, u16 bc, u8 sizey)
{           
    static char last_chars[MAX_INT_CACHE_SLOTS][MAX_INT_CHARS];
    static u16 last_x[MAX_INT_CACHE_SLOTS];
    static u16 last_y[MAX_INT_CACHE_SLOTS];
    static u8 slot_count = 0;
    static u8 initialized = 0;
    
    u8 i, sizex;
    sizex = sizey / 2;
    
    // 首次初始化
    if(!initialized)
    {
        for(i = 0; i < MAX_INT_CACHE_SLOTS; i++)
        {
            last_x[i] = 0xFFFF;
            last_y[i] = 0xFFFF;
            last_chars[i][0] = '\0';
        }
        initialized = 1;
    }
    
    // 查找或分配缓存槽位
    u8 slot = 0xFF;
    for(i = 0; i < MAX_INT_CACHE_SLOTS; i++)
    {
        if(last_x[i] == x && last_y[i] == y)
        {
            slot = i;
            break;
        }
    }
    
    if(slot == 0xFF)  // 新位置，分配一个槽位
    {
        if(slot_count < MAX_INT_CACHE_SLOTS)
        {
            slot = slot_count;
            slot_count++;
        }
        else
        {
            // 所有槽位已满，找第一个重用
            slot = 0;
        }
        last_x[slot] = x;
        last_y[slot] = y;
        last_chars[slot][0] = '\0';
    }
    
    char new_chars[MAX_INT_CHARS];
    u8 char_idx = 0;
    
    int abs_num;
    
    // 处理负号
    if(num < 0)
    {
        new_chars[char_idx++] = '-';
        abs_num = -num;
    }
    else
    {
        abs_num = num;
    }
    
    // 处理数字部分 - 先转换为字符串
    if(abs_num == 0)
    {
        new_chars[char_idx++] = '0';
    }
    else
    {
        // 计算位数
        u8 len = 0;
        int temp = abs_num;
        while(temp > 0)
        {
            len++;
            temp /= 10;
        }
        
        // 从高位到低位生成字符
        for(i = 0; i < len; i++)
        {
            u8 digit = (abs_num / mypow(10, len - i - 1)) % 10;
            new_chars[char_idx++] = '0' + digit;
        }
    }
    
    new_chars[char_idx] = '\0';
    
    // 更新显示（只刷变化的部分）
    u8 actual_len = char_idx;
    for(i = 0; i < actual_len; i++)
    {
        if(new_chars[i] != last_chars[slot][i])
        {
            LCD_ShowChar(x + i * sizex, y, new_chars[i], fc, bc, sizey, 0);
        }
    }
    
    // 清除多余字符（当新数字比旧数字短时）
    u8 last_len = 0;
    while(last_chars[slot][last_len] != '\0' && last_len < MAX_INT_CHARS)
    {
        last_len++;
    }
    
    if(actual_len < last_len)
    {
        for(i = actual_len; i < last_len; i++)
        {
            LCD_ShowChar(x + i * sizex, y, ' ', fc, bc, sizey, 0);
        }
    }
    
    // 更新缓存
    for(i = 0; i < actual_len; i++)
    {
        last_chars[slot][i] = new_chars[i];
    }
    last_chars[slot][actual_len] = '\0';
}

/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp,sizex;
	u16 num1;
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


/******************************************************************************
      函数说明：显示浮点数（支持负数、可配置小数位数）
      入口数据：x,y显示坐标
                num 要显示小数变量
                int_len 整数部分位数（不足补空格）
                dec_len 小数部分位数（1-4位）
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
#define MAX_CACHE_SLOTS 10   //可显示槽位数量
#define MAX_CHAR_LEN 20

void LCD_ShowFloatNumEx(u16 x, u16 y, float num, u8 dec_len, u16 fc, u16 bc, u8 sizey)
{           
    static char last_chars[MAX_CACHE_SLOTS][MAX_CHAR_LEN];
    static u16 last_x[MAX_CACHE_SLOTS];
    static u16 last_y[MAX_CACHE_SLOTS];
    static u8 slot_count = 0;
    static u8 initialized = 0;
    
    u8 i, sizex;
    sizex = sizey / 2;
    
    // 首次初始化
    if(!initialized)
    {
        for(i = 0; i < MAX_CACHE_SLOTS; i++)
        {
            last_x[i] = 0xFFFF;
            last_y[i] = 0xFFFF;
            last_chars[i][0] = '\0';
        }
        initialized = 1;
    }
    
    // 查找或分配缓存槽位
    u8 slot = 0xFF;
    for(i = 0; i < MAX_CACHE_SLOTS; i++)
    {
        if(last_x[i] == x && last_y[i] == y)
        {
            slot = i;
            break;
        }
    }
    
    if(slot == 0xFF)  // 新位置，分配一个槽位
    {
        if(slot_count < MAX_CACHE_SLOTS)
        {
            slot = slot_count;
            slot_count++;
        }
        else
        {
            // 所有槽位已满，找一个最旧的重用（简单轮转）
            slot = 0;  // 或实现更复杂的LRU
        }
        last_x[slot] = x;
        last_y[slot] = y;
        last_chars[slot][0] = '\0';
    }
    
    char new_chars[MAX_CHAR_LEN];
    u8 char_idx = 0;
    
    // 处理符号
    if(num < 0)
    {
        new_chars[char_idx++] = '-';
        num = -num;
    }
    else
    {
        new_chars[char_idx++] = ' ';
    }
    
    // 四舍五入
    u32 multiplier = mypow(10, dec_len);
    u32 scaled_num = (u32)(num * multiplier + 0.5f);
    
    u32 int_part = scaled_num / multiplier;
    u32 dec_part = scaled_num % multiplier;
    
    // 处理整数部分
    if(int_part >= 100)
    {
        new_chars[char_idx++] = '0' + (int_part / 100) % 10;
        new_chars[char_idx++] = '0' + (int_part / 10) % 10;
        new_chars[char_idx++] = '0' + int_part % 10;
    }
    else if(int_part >= 10)
    {
        new_chars[char_idx++] = '0' + (int_part / 10) % 10;
        new_chars[char_idx++] = '0' + int_part % 10;
    }
    else
    {
        new_chars[char_idx++] = '0' + int_part % 10;
    }
    
    // 小数点
    new_chars[char_idx++] = '.';
    
    // 小数部分
    for(i = 0; i < dec_len; i++)
    {
        u32 divisor = mypow(10, dec_len - i - 1);
        u32 digit = (dec_part / divisor) % 10;
        new_chars[char_idx++] = '0' + digit;
    }
    new_chars[char_idx] = '\0';
    
    // 更新显示（只刷变化的部分）
    u8 actual_len = char_idx;
    for(i = 0; i < actual_len; i++)
    {
        if(new_chars[i] != last_chars[slot][i])
        {
            LCD_ShowChar(x + i * sizex, y, new_chars[i], fc, bc, sizey, 0);
        }
    }
    
    // 清除多余字符
    u8 last_len = 0;
    while(last_chars[slot][last_len] != '\0' && last_len < MAX_CHAR_LEN)
    {
        last_len++;
    }
    
    if(actual_len < last_len)
    {
        for(i = actual_len; i < last_len; i++)
        {
            LCD_ShowChar(x + i * sizex, y, ' ', fc, bc, sizey, 0);
        }
    }
    
    // 更新缓存
    for(i = 0; i < actual_len; i++)
    {
        last_chars[slot][i] = new_chars[i];
    }
    last_chars[slot][actual_len] = '\0';
}

/******************************************************************************
      函数说明：显示带符号的两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowNegativeFloat(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc, u8 sizey)
{
    u8 sizex = sizey / 2;
    
    if(num < 0)
    {
        LCD_ShowChar(x, y, '-', fc, bc, sizey, 0);  // 显示负号
        x += sizex;
        num = -num;
    }
    
    // 调用原函数显示正数部分
    LCD_ShowFloatNum1(x, y, num, len, fc, bc, sizey);
}


/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组    
      返回值：  无
******************************************************************************/
void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[])
{
	u16 i,j;
	u32 k=0;
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


