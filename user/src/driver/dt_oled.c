/**
 * @file dt_oled.c
 * @brief SSD1306 OLED显示屏驱动实现
 *        通过IIC接口通信，使用帧缓冲机制减少I2C传输次数。
 *        支持6x8点阵字符显示、整数/十六进制/浮点数显示、
 *        脏页刷新机制（仅更新有变化的页面，提高刷新效率）。
 */

#include "driver/dt_oled.h"

#include <string.h>

#define DT_OLED_CMD  0x00   /**< I2C控制字节：发送命令 */
#define DT_OLED_DATA 0x40   /**< I2C控制字节：发送数据 */

/**
 * @brief 6x8 ASCII字符点阵字库
 *        每个字符用6个字节表示，每列1字节，字节的每个bit对应一行像素。
 *        索引0=空格，索引1~95对应ASCII码0x21~0x7E。
 *        最后一行为空行，用于字符间间距。
 */
static const uint8_t font_6x8[][6] = {
    /* 0x00 空格 */    {0x00,0x00,0x00,0x00,0x00,0x00},
    /* 0x01 ! */       {0x00,0x00,0x5F,0x00,0x00,0x00},
    /* 0x02 " */       {0x00,0x07,0x00,0x07,0x00,0x00},
    /* 0x03 # */       {0x14,0x7F,0x14,0x7F,0x14,0x00},
    /* 0x04 $ */       {0x24,0x2A,0x7F,0x2A,0x12,0x00},
    /* 0x05 % */       {0x23,0x13,0x08,0x64,0x62,0x00},
    /* 0x06 & */       {0x36,0x49,0x55,0x22,0x50,0x00},
    /* 0x07 ' */       {0x00,0x05,0x03,0x00,0x00,0x00},
    /* 0x08 ( */       {0x00,0x1C,0x22,0x41,0x00,0x00},
    /* 0x09 ) */       {0x00,0x41,0x22,0x1C,0x00,0x00},
    /* 0x0A * */       {0x08,0x2A,0x1C,0x2A,0x08,0x00},
    /* 0x0B + */       {0x08,0x08,0x3E,0x08,0x08,0x00},
    /* 0x0C , */       {0x00,0x50,0x30,0x00,0x00,0x00},
    /* 0x0D - */       {0x08,0x08,0x08,0x08,0x08,0x00},
    /* 0x0E . */       {0x00,0x60,0x60,0x00,0x00,0x00},
    /* 0x0F / */       {0x20,0x10,0x08,0x04,0x02,0x00},
    /* 0x10 0 */       {0x3E,0x51,0x49,0x45,0x3E,0x00},
    /* 0x11 1 */       {0x00,0x42,0x7F,0x40,0x00,0x00},
    /* 0x12 2 */       {0x42,0x61,0x51,0x49,0x46,0x00},
    /* 0x13 3 */       {0x21,0x41,0x45,0x4B,0x31,0x00},
    /* 0x14 4 */       {0x18,0x14,0x12,0x7F,0x10,0x00},
    /* 0x15 5 */       {0x27,0x45,0x45,0x45,0x39,0x00},
    /* 0x16 6 */       {0x3C,0x4A,0x49,0x49,0x30,0x00},
    /* 0x17 7 */       {0x01,0x71,0x09,0x05,0x03,0x00},
    /* 0x18 8 */       {0x36,0x49,0x49,0x49,0x36,0x00},
    /* 0x19 9 */       {0x06,0x49,0x49,0x29,0x1E,0x00},
    /* 0x1A : */       {0x00,0x36,0x36,0x00,0x00,0x00},
    /* 0x1B ; */       {0x00,0x56,0x36,0x00,0x00,0x00},
    /* 0x1C < */       {0x00,0x08,0x14,0x22,0x41,0x00},
    /* 0x1D = */       {0x14,0x14,0x14,0x14,0x14,0x00},
    /* 0x1E > */       {0x41,0x22,0x14,0x08,0x00,0x00},
    /* 0x1F ? */       {0x02,0x01,0x51,0x09,0x06,0x00},
    /* 0x20 @ */       {0x32,0x49,0x79,0x41,0x3E,0x00},
    /* 0x21 A */       {0x7E,0x11,0x11,0x11,0x7E,0x00},
    /* 0x22 B */       {0x7F,0x49,0x49,0x49,0x36,0x00},
    /* 0x23 C */       {0x3E,0x41,0x41,0x41,0x22,0x00},
    /* 0x24 D */       {0x7F,0x41,0x41,0x22,0x1C,0x00},
    /* 0x25 E */       {0x7F,0x49,0x49,0x49,0x41,0x00},
    /* 0x26 F */       {0x7F,0x09,0x09,0x01,0x01,0x00},
    /* 0x27 G */       {0x3E,0x41,0x41,0x51,0x32,0x00},
    /* 0x28 H */       {0x7F,0x08,0x08,0x08,0x7F,0x00},
    /* 0x29 I */       {0x00,0x41,0x7F,0x41,0x00,0x00},
    /* 0x2A J */       {0x20,0x40,0x41,0x3F,0x01,0x00},
    /* 0x2B K */       {0x7F,0x08,0x14,0x22,0x41,0x00},
    /* 0x2C L */       {0x7F,0x40,0x40,0x40,0x40,0x00},
    /* 0x2D M */       {0x7F,0x02,0x04,0x02,0x7F,0x00},
    /* 0x2E N */       {0x7F,0x04,0x08,0x10,0x7F,0x00},
    /* 0x2F O */       {0x3E,0x41,0x41,0x41,0x3E,0x00},
    /* 0x30 P */       {0x7F,0x09,0x09,0x09,0x06,0x00},
    /* 0x31 Q */       {0x3E,0x41,0x51,0x21,0x5E,0x00},
    /* 0x32 R */       {0x7F,0x09,0x19,0x29,0x46,0x00},
    /* 0x33 S */       {0x46,0x49,0x49,0x49,0x31,0x00},
    /* 0x34 T */       {0x01,0x01,0x7F,0x01,0x01,0x00},
    /* 0x35 U */       {0x3F,0x40,0x40,0x40,0x3F,0x00},
    /* 0x36 V */       {0x1F,0x20,0x40,0x20,0x1F,0x00},
    /* 0x37 W */       {0x7F,0x20,0x18,0x20,0x7F,0x00},
    /* 0x38 X */       {0x63,0x14,0x08,0x14,0x63,0x00},
    /* 0x39 Y */       {0x03,0x04,0x78,0x04,0x03,0x00},
    /* 0x3A Z */       {0x61,0x51,0x49,0x45,0x43,0x00},
    /* 0x3B [ */       {0x00,0x00,0x7F,0x41,0x41,0x00},
    /* 0x3C \ */       {0x02,0x04,0x08,0x10,0x20,0x00},
    /* 0x3D ] */       {0x41,0x41,0x7F,0x00,0x00,0x00},
    /* 0x3E ^ */       {0x04,0x02,0x01,0x02,0x04,0x00},
    /* 0x3F _ */       {0x40,0x40,0x40,0x40,0x40,0x00},
    /* 0x40 ` */       {0x00,0x01,0x02,0x04,0x00,0x00},
    /* 0x41 a */       {0x20,0x54,0x54,0x54,0x78,0x00},
    /* 0x42 b */       {0x7F,0x48,0x44,0x44,0x38,0x00},
    /* 0x43 c */       {0x38,0x44,0x44,0x44,0x20,0x00},
    /* 0x44 d */       {0x38,0x44,0x44,0x48,0x7F,0x00},
    /* 0x45 e */       {0x38,0x54,0x54,0x54,0x18,0x00},
    /* 0x46 f */       {0x08,0x7E,0x09,0x01,0x02,0x00},
    /* 0x47 g */       {0x08,0x14,0x54,0x54,0x3C,0x00},
    /* 0x48 h */       {0x7F,0x08,0x04,0x04,0x78,0x00},
    /* 0x49 i */       {0x00,0x44,0x7D,0x40,0x00,0x00},
    /* 0x4A j */       {0x20,0x40,0x44,0x3D,0x00,0x00},
    /* 0x4B k */       {0x00,0x7F,0x10,0x28,0x44,0x00},
    /* 0x4C l */       {0x00,0x41,0x7F,0x40,0x00,0x00},
    /* 0x4D m */       {0x7C,0x04,0x18,0x04,0x78,0x00},
    /* 0x4E n */       {0x7C,0x08,0x04,0x04,0x78,0x00},
    /* 0x4F o */       {0x38,0x44,0x44,0x44,0x38,0x00},
    /* 0x50 p */       {0x7C,0x14,0x14,0x14,0x08,0x00},
    /* 0x51 q */       {0x08,0x14,0x14,0x18,0x7C,0x00},
    /* 0x52 r */       {0x7C,0x08,0x04,0x04,0x08,0x00},
    /* 0x53 s */       {0x48,0x54,0x54,0x54,0x20,0x00},
    /* 0x54 t */       {0x04,0x3F,0x44,0x40,0x20,0x00},
    /* 0x55 u */       {0x3C,0x40,0x40,0x20,0x7C,0x00},
    /* 0x56 v */       {0x1C,0x20,0x40,0x20,0x1C,0x00},
    /* 0x57 w */       {0x3C,0x40,0x30,0x40,0x3C,0x00},
    /* 0x58 x */       {0x44,0x28,0x10,0x28,0x44,0x00},
    /* 0x59 y */       {0x0C,0x50,0x50,0x50,0x3C,0x00},
    /* 0x5A z */       {0x44,0x64,0x54,0x4C,0x44,0x00},
    /* 0x5B { */       {0x00,0x08,0x36,0x41,0x00,0x00},
    /* 0x5C | */       {0x00,0x00,0x7F,0x00,0x00,0x00},
    /* 0x5D } */       {0x00,0x41,0x36,0x08,0x00,0x00},
    /* 0x5E ~ */       {0x08,0x04,0x08,0x10,0x08,0x00},
};

/**
 * @brief 向OLED发送命令序列（IIC写入，控制字节为0x00）
 * @param cfg OLED配置结构体指针
 * @param commands 命令缓冲区指针
 * @param length 命令长度（字节）
 * @return IIC操作状态
 */
static soft_iic_status_enum dt_oled_write_cmds(dt_oled_config_t *cfg,
    const uint8_t *commands, uint32_t length)
{
    const uint8_t control = DT_OLED_CMD;

    /* 使用拼接写入：先发控制字节(0x00=命令)，再发命令数据 */
    return soft_iic_write_splicing_array_checked(&cfg->iic, &control, 1u,
        commands, length);
}

/**
 * @brief 初始化OLED显示屏
 *        发送SSD1306初始化命令序列，清空帧缓冲，标记所有页为脏
 * @param cfg OLED配置结构体指针
 *
 * 初始化命令说明：
 *   0xAE - 关闭显示
 *   0xD5 0x80 - 设置显示时钟分频/振荡频率
 *   0xA8 0x3F - 设置多路复用比（64行）
 *   0xD3 0x00 - 设置显示偏移（无偏移）
 *   0x40 - 设置显示起始行（行0）
 *   0x8D 0x14 - 启用电荷泵
 *   0x20 0x00 - 水平寻址模式
 *   0xA1 - 左右镜像（列地址127映射到SEG0）
 *   0xC8 - 上下翻转（COM从COM[N-1]到COM0扫描）
 *   0xDA 0x12 - 设置COM引脚硬件配置
 *   0x81 0xCF - 设置对比度
 *   0xD9 0xF1 - 设置预充电周期
 *   0xDB 0x40 - 设置VCOMH电压
 *   0xA4 - 全局显示：正常显示（不忽略RAM内容）
 *   0xA6 - 正常显示（非反相）
 *   0xAF - 打开显示
 */
void dt_oled_init(dt_oled_config_t *cfg)
{
    static const uint8_t init_commands[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0xAF
    };

    system_delay_ms(100); /* 等待OLED上电稳定 */
    dt_oled_write_cmds(cfg, init_commands, sizeof(init_commands));
    memset(cfg->framebuffer, 0, sizeof(cfg->framebuffer)); /* 清空帧缓冲 */
    cfg->dirty_pages = 0xFFu; /* 所有页标记为脏，下次刷新全部更新 */
}

/**
 * @brief 设置OLED写入位置（页地址+列地址）
 * @param cfg OLED配置结构体指针
 * @param x 列地址（0~127）
 * @param y 页地址（0~7，每页8行像素）
 * @note SSD1306的页寻址模式中，设置位置需要3个命令：
 *       0xB0+page - 设置页地址
 *       0x10+(col>>4) - 设置列地址高4位
 *       col&0x0F - 设置列地址低4位
 */
void dt_oled_set_pos(dt_oled_config_t *cfg, uint8_t x, uint8_t y)
{
    uint8_t commands[3];

    commands[0] = (uint8_t)(0xB0u + y);           /* 设置页地址（0xB0~0xB7） */
    commands[1] = (uint8_t)(((x & 0xF0u) >> 4) | 0x10u); /* 列高4位 */
    commands[2] = x & 0x0Fu;                      /* 列低4位 */
    dt_oled_write_cmds(cfg, commands, sizeof(commands));
}

/**
 * @brief 清屏（所有像素熄灭）
 * @param cfg OLED配置结构体指针
 */
void dt_oled_clear(dt_oled_config_t *cfg)
{
    dt_oled_fill(cfg, 0x00u); /* 全屏填充0x00 */
}

/**
 * @brief 全屏填充指定数据
 *        优化：仅当页面数据实际发生改变时才写入缓冲和标记脏页，
 *        避免不必要的帧缓冲写入操作。
 * @param cfg OLED配置结构体指针
 * @param data 填充数据
 */
void dt_oled_fill(dt_oled_config_t *cfg, uint8_t data)
{
    uint8_t page;

    for (page = 0u; page < DT_OLED_PAGE_COUNT; page++)
    {
        uint8_t x;
        bool changed = false;

        /* 检查当前页是否已有变化 */
        for (x = 0u; x < DT_OLED_WIDTH; x++)
        {
            if (cfg->framebuffer[page][x] != data)
            {
                changed = true;
                break;
            }
        }
        if (changed)
        {
            memset(cfg->framebuffer[page], data, DT_OLED_WIDTH);
            dt_oled_mark_page_dirty(cfg, page);
        }
    }
}

/**
 * @brief 在指定位置显示一个6x8像素的ASCII字符
 *        只将变化的数据写入帧缓冲（优化：避免重复写相同数据）
 * @param cfg OLED配置结构体指针
 * @param x 起始列（0~127）
 * @param y 页地址（0~7）
 * @param ch 要显示的字符（仅支持ASCII 0x20~0x7E，其他显示为空格）
 */
void dt_oled_show_char(dt_oled_config_t *cfg, uint8_t x, uint8_t y, char ch)
{
    uint8_t i;
    const uint8_t *glyph;

    /* 边界检查 */
    if (y >= DT_OLED_PAGE_COUNT || x >= DT_OLED_WIDTH)
    {
        return;
    }

    /* 从字库中获取字符点阵，不可打印字符显示为空格 */
    glyph = (ch >= ' ' && ch <= '~') ? font_6x8[ch - ' '] : font_6x8[0];

    /* 逐个写入6个字节的点阵数据（每字节对应一列） */
    for (i = 0u; i < 6u && (uint16_t)x + i < DT_OLED_WIDTH; i++)
    {
        /* 仅有变化时才写入缓冲，避免不必要的操作 */
        if (cfg->framebuffer[y][x + i] != glyph[i])
        {
            cfg->framebuffer[y][x + i] = glyph[i];
            dt_oled_mark_page_dirty(cfg, y); /* 标记该页为脏 */
        }
    }
}

/**
 * @brief 在指定位置显示字符串
 *        超出屏幕宽度自动换行到下一页
 * @param cfg OLED配置结构体指针
 * @param x 起始列
 * @param y 起始页
 * @param str 以'\0'结尾的字符串
 */
void dt_oled_show_string(dt_oled_config_t *cfg, uint8_t x, uint8_t y, const char *str)
{
    while (*str != '\0' && y < DT_OLED_PAGE_COUNT)
    {
        /* 超出列宽则换行 */
        if (x >= DT_OLED_WIDTH)
        {
            x = 0u;
            y++;
            if (y >= DT_OLED_PAGE_COUNT)
            {
                break; /* 已超出屏幕范围 */
            }
        }
        dt_oled_show_char(cfg, x, y, *str);
        x = (uint8_t)(x + 6u); /* 字符宽度6像素 */
        /* 检查下一个字符是否需要换行（6像素宽+6像素间距） */
        if (x > DT_OLED_WIDTH - 6)
        {
            x = 0u;
            y++;
        }
        str++;
    }
}

/**
 * @brief 将指定页标记为脏（需要刷新到屏幕）
 * @param cfg OLED配置结构体指针
 * @param page 页索引（0~7）
 */
void dt_oled_mark_page_dirty(dt_oled_config_t *cfg, uint8_t page)
{
    if (page < DT_OLED_PAGE_COUNT)
    {
        cfg->dirty_pages |= (uint8_t)(1u << page); /* 置位对应的位 */
    }
}

/**
 * @brief mark_line_dirty 的别名（兼容不同命名习惯）
 * @param cfg OLED配置结构体指针
 * @param line 行/页索引
 */
void dt_oled_mark_line_dirty(dt_oled_config_t *cfg, uint8_t line)
{
    dt_oled_mark_page_dirty(cfg, line);
}

/**
 * @brief 刷新指定页（通过IIC将帧缓冲数据传输到OLED）
 *        先设置写入位置，再发送128字节的显示数据。
 *        发送成功后清除该页的脏标记。
 * @param cfg OLED配置结构体指针
 * @param page 要刷新的页索引
 */
void dt_oled_refresh_page(dt_oled_config_t *cfg, uint8_t page)
{
    const uint8_t control = DT_OLED_DATA;

    if (page >= DT_OLED_PAGE_COUNT)
    {
        return;
    }

    /* 设置写入位置到指定页的起始地址 */
    dt_oled_set_pos(cfg, 0u, page);
    if(SOFT_IIC_STATUS_OK != soft_iic_get_last_error(&cfg->iic))
    {
        return; /* IIC错误，停止刷新 */
    }

    /* 发送整页数据：控制字节(0x40=数据) + 128字节帧缓冲 */
    if(SOFT_IIC_STATUS_OK == soft_iic_write_splicing_array_checked(
            &cfg->iic, &control, 1u, cfg->framebuffer[page], DT_OLED_WIDTH))
    {
        cfg->dirty_pages &= (uint8_t)~(uint8_t)(1u << page); /* 清除脏标记 */
    }
}

/**
 * @brief refresh_line 的别名（兼容不同命名习惯）
 * @param cfg OLED配置结构体指针
 * @param line 行/页索引
 */
void dt_oled_refresh_line(dt_oled_config_t *cfg, uint8_t line)
{
    dt_oled_refresh_page(cfg, line);
}

/**
 * @brief 只刷新第一个遇到脏页（每次仅刷新一页）
 *        用于分散刷新，避免单次IIC传输过长时间阻塞系统
 * @param cfg OLED配置结构体指针
 */
void dt_oled_refresh_one_dirty(dt_oled_config_t *cfg)
{
    uint8_t page;

    for (page = 0u; page < DT_OLED_PAGE_COUNT; page++)
    {
        if ((cfg->dirty_pages & (uint8_t)(1u << page)) != 0u)
        {
            dt_oled_refresh_page(cfg, page);
            return; /* 每次只刷一页 */
        }
    }
}

/**
 * @brief 刷新所有脏页到OLED
 * @param cfg OLED配置结构体指针
 */
void dt_oled_refresh_dirty(dt_oled_config_t *cfg)
{
    uint8_t page;

    for (page = 0u; page < DT_OLED_PAGE_COUNT; page++)
    {
        if ((cfg->dirty_pages & (uint8_t)(1u << page)) != 0u)
        {
            dt_oled_refresh_page(cfg, page);
        }
    }
}

/**
 * @brief 任务调度器回调封装（刷新所有脏页）
 * @param now_ms 当前时间戳（本函数中未使用）
 * @param context 指向dt_oled_config_t的指针
 */
void dt_oled_refresh_task(uint32_t now_ms, void *context)
{
    (void)now_ms;
    if (context != NULL)
    {
        dt_oled_refresh_dirty((dt_oled_config_t *)context);
    }
}

/**
 * @brief 显示整数（支持负数）
 *        负数显示"-"符号加数字，正数直接显示数字
 * @param cfg OLED配置结构体指针
 * @param x 起始列
 * @param y 起始页
 * @param num 整数（int32_t范围）
 * @param len 数字位数（不含符号位）
 * @note 若num位数超过len，高位数字被截断
 */
void dt_oled_show_num(dt_oled_config_t *cfg, uint8_t x, uint8_t y, int32_t num, uint8_t len)
{
    char buf[12];
    uint32_t magnitude;
    uint8_t i;
    uint8_t start = 0;

    if (len > 10u) len = 10u;

    /* 处理负数：取绝对值，预留'-'位置 */
    magnitude = (num < 0) ? (uint32_t)(-(int64_t)num) : (uint32_t)num;
    if (num < 0)
    {
        buf[0] = '-';
        start = 1;
    }

    /* 从个位开始逐位分解数字 */
    for (i = 0; i < len; i++)
    {
        buf[len - 1 - i + start] = '0' + (uint8_t)(magnitude % 10u);
        magnitude /= 10u;
    }
    buf[len + start] = '\0'; /* 字符串结束 */
    dt_oled_show_string(cfg, x, y, buf);
}

/**
 * @brief 显示十六进制数
 * @param cfg OLED配置结构体指针
 * @param x 起始列
 * @param y 起始页
 * @param num 32位无符号整数
 * @param len 显示的十六进制位数（最多8位）
 * @note 不显示"0x"前缀
 */
void dt_oled_show_hex(dt_oled_config_t *cfg, uint8_t x, uint8_t y, uint32_t num, uint8_t len)
{
    char buf[12];
    const char hex[] = "0123456789ABCDEF";
    uint8_t i;

    if (len > 8u) len = 8u;

    /* 从最低4位开始，逐位转换为十六进制字符 */
    for (i = 0; i < len; i++)
    {
        buf[len - 1 - i] = hex[num & 0x0F];
        num >>= 4;
    }
    buf[len] = '\0';
    dt_oled_show_string(cfg, x, y, buf);
}

/**
 * @brief 显示浮点数
 *        支持正负数，四舍五入到指定位数
 * @param cfg OLED配置结构体指针
 * @param x 起始列
 * @param y 起始页
 * @param num 浮点数
 * @param int_len 整数部分位数（最多10位）
 * @param dec_len 小数部分位数（最多6位）
 */
void dt_oled_show_float(dt_oled_config_t *cfg, uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len)
{
    int32_t int_part;
    uint32_t dec_part;
    uint32_t pow10 = 1;
    uint8_t i;

    if (int_len > 10u) int_len = 10u;
    if (dec_len > 6u) dec_len = 6u;

    /* 处理负数 */
    if (num < 0)
    {
        dt_oled_show_char(cfg, x, y, '-');
        x += 6;     /* 跳过负号占位 */
        num = -num;
    }

    /* 计算10^dec_len，用于提取小数部分 */
    for (i = 0; i < dec_len; i++) pow10 *= 10;

    /* 分离整数和小数部分，小数部分四舍五入 */
    int_part = (int32_t)num;
    dec_part = (uint32_t)((num - (float)int_part) * (float)pow10 + 0.5f);
    if (dec_part >= pow10) { int_part++; dec_part = 0; } /* 处理进位 */

    /* 显示整数部分 + 小数点 + 小数部分 */
    dt_oled_show_num(cfg, x, y, int_part, int_len);
    x += int_len * 6;
    dt_oled_show_char(cfg, x, y, '.');
    x += 6;
    dt_oled_show_num(cfg, x, y, (int32_t)dec_part, dec_len);
}
