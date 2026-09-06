#include "wit_c_sdk.h"
//传感器往Arduino的串口发数据的函数的指针，初始值为0
static SerialWrite p_WitSerialWriteFunc = NULL;
static RegUpdateCb p_WitRegUpdateCbFunc = NULL;
static DelaymsCb p_WitDelaymsFunc = NULL;

static WitI2cWrite p_WitI2cWriteFunc = NULL; //I2C写数据用，我用不到
static WitI2cRead p_WitI2cReadFunc = NULL; //I2C读数据用，我用不到
static CanWrite p_WitCanWriteFunc = NULL; //CAN写数据用，我用不到

static uint8_t s_ucWitDataBuff[WIT_DATA_BUFF_SIZE]; //数据缓存
static uint8_t s_ucAddr = 0xff;
static uint32_t s_uiWitDataCnt = 0; //数据计数器
static uint32_t s_uiProtoclo = 0; //通信协议选择
static uint32_t s_uiReadRegIndex = 0; //读取寄存器的时候，要读取的寄存器的地址

//sReg是一个数组
//数组的元素一共有0x90个，每个元素是16位的二进制数
//数组的每个位置存放什么数据通过REG.h当中的宏定义加以说明
int16_t sReg[REGSIZE];


#define FuncW 0x06
#define FuncR 0x03

//CRC校验表的高8位
static const uint8_t __auchCRCHi[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};
//CRC校验表的低8位
static const uint8_t __auchCRCLo[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};
//循环冗余校验算法
//传入变量为 1.需要计算校验码的数据的指针 2.数据的长度（字节为单位）
static uint16_t __CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF; //校验码高8位
    uint8_t uchCRCLo = 0xFF; //校验码低8位
    uint8_t uIndex;
    int i;
    for (i=0; i<usDataLen; i++)
    {
        uIndex = uchCRCHi ^ puchMsg[i];
        uchCRCHi = uchCRCLo ^ __auchCRCHi[uIndex];
        uchCRCLo = __auchCRCLo[uIndex] ;
    }
    //最后回传16位校验码
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo) ;
}
//计算一个数组的校验和
//传入变量为 1.需要计算校验和的数据的指针 2.数据的长度（字节为单位）
static uint8_t __CaliSum(uint8_t *data, uint32_t len)
{
    //功能是把所有的数据求和，最后得到一个八位校验值
    //data是数组首个元素的地址
    uint32_t i;
    uint8_t ucCheck = 0;
    for(i=0; i<len; i++) ucCheck += *(data + i);
    return ucCheck;
}



//传感器初始化
//传入传感器的通信方式和一个地址
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr)
{
	if(uiProtocol > WIT_PROTOCOL_I2C)return WIT_HAL_INVAL;
    //选择通信协议
    s_uiProtoclo = uiProtocol;
    //选择地址（还不知道是什么地址）
    s_ucAddr = ucAddr;
    //数据计数
    s_uiWitDataCnt = 0;
    return WIT_HAL_OK;
}
//传感器重置
//主要是把一些通信相关的函数重置
//由于不同的通信模式，传感器发送数据的方式不一样，并且发送到哪个位置最好是能在用户层被修改，所以没有写入到sdk当中
void WitDeInit(void)
{
    p_WitSerialWriteFunc = NULL;
    p_WitI2cWriteFunc = NULL;
    p_WitI2cReadFunc = NULL;
    p_WitCanWriteFunc = NULL;
    p_WitRegUpdateCbFunc = NULL;
    s_ucAddr = 0xff;
    s_uiWitDataCnt = 0;
    s_uiProtoclo = 0;
}



//修改全局变量p_WitSerialWriteFunc的值，传入一个外部定义的函数的指针，调用skd中的WitWriteReg()和WitReadReg()函数时，实际内部会调用外部定义的函数
int32_t WitSerialWriteRegister(SerialWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}
//修改全局变量p_WitRegUpdateCbFunc的值，传入一个外部定义的函数的指针，调用skd中的WitWriteReg()和WitReadReg()函数时，实际内部会调用外部定义的函数
int32_t WitRegisterCallBack(RegUpdateCb update_func)
{
    if(!update_func)return WIT_HAL_INVAL;
    p_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}
//修改全局变量p_WitDelaymsFunc的值，传入一个外部定义的函数的指针，用于延时处理，主要是在修改模块寄存器值的时候需要延时（使用文档中写到）
int32_t WitDelayMsRegister(DelaymsCb delayms_func)
{
    if(!delayms_func)return WIT_HAL_INVAL;
    p_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}



//该函数是在sdk内部被调用的，并不直接使用
//串口接收到传感器发来的数据之后，会生成数据索引ucIndex，数据内容p_data，数据长度（uiLen）
//CopeWitData函数会解析上述内容并且把数据复制到sREG数组中
//同时调用“数组数据更新提示”函数，让用户那边的某个指示变量更新，用于表示某些寄存器的数据已经更新
static void CopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen)
{
    uint32_t uiReg1 = 0, uiReg2 = 0, uiReg1Len = 0, uiReg2Len = 0;
    uint16_t *p_usReg1Val = p_data;
    uint16_t *p_usReg2Val = p_data+3;
    uiReg1Len = 4;
    switch(ucIndex)
    {
        //根据索引知道传感器这边给出的数据类型，然后设置该数据应该放在哪个寄存器里，并且更新数据长度
        case WIT_TIME:  uiReg1 = YYMM;	break; //时间，由于uiReg1Len默认是4，所以不需要修改
        case WIT_ACC:   uiReg1 = AX;    uiReg1Len = 3;  uiReg2 = TEMP;  uiReg2Len = 1;  break; //加速度
        case WIT_GYRO:  uiReg1 = GX;  uiLen = 3;break; //角速度
        case WIT_ANGLE: uiReg1 = Roll;  uiReg1Len = 3;  uiReg2 = VERSION;  uiReg2Len = 1;  break; //角度
        case WIT_MAGNETIC: uiReg1 = HX;  uiLen = 3;break; //磁场
        case WIT_DPORT: uiReg1 = D0Status;  break; //端口D
        case WIT_GPS:   uiReg1 = LonL;  break; //经度纬度
        case WIT_VELOCITY: uiReg1 = GPSHeight;  break; //速度，存GPS高度、GPS航向角和速度
        case WIT_QUATER:    uiReg1 = q0;  break; //四元数
        case WIT_GSA:   uiReg1 = SVNUM;  break; //定位精度，存卫星数量、位置精度、水平精度、垂直精度
        case WIT_REGVALUE:  uiReg1 = s_uiReadRegIndex;  break; //读别的寄存器中的数据
		default: return;
    }
    if(uiLen == 3) //对于角速度和磁场会进行这一条判断
    {
        uiReg1Len = 3;
        uiReg2Len = 0;
    }
    if(uiReg1Len)
	{
        //这是在string.h中定义的，用于高效的复制数据
        //第一个参数是目标内存区域的起始地址，这里用了对数组取地址
        //第二个参数是源内存区域的起始地址，传感器传过来的数据是随机存储的，所以需要知道地址
        //第三个参数uiReg1Len进行了*2操作，是因为传递的数据是uint16_t
		memcpy(&sReg[uiReg1], p_usReg1Val, uiReg1Len<<1);
        //然后调用这个“寄存器数据更新提示”函数，并且传入更新的寄存器索引和数据个数
		p_WitRegUpdateCbFunc(uiReg1, uiReg1Len);
	}
    if(uiReg2Len) //如果这里的数据有多种类型的，那么uiReg2Len的值是不为0的，继续调用“寄存器数据更新提示”函数
	{
		memcpy(&sReg[uiReg2], p_usReg2Val, uiReg2Len<<1);
		p_WitRegUpdateCbFunc(uiReg2, uiReg2Len);
	}
}
//函数可以处理Arduino从串口读到的从传感器发来的数据
void WitSerialDataIn(uint8_t ucData)
{//ucData是传入的一个字节的数据
    //usCRC16储存CRC校验值
    uint16_t usCRC16;
    //usTemp储存中间计算结果
    uint16_t usTemp;
    //循环变量
    uint16_t i;
    //usData数组储存解析后的数据
    uint16_t usData[4];
    //储存校验和
    uint8_t ucSum;
    if(p_WitRegUpdateCbFunc == NULL)return ;
    s_ucWitDataBuff[s_uiWitDataCnt++] = ucData;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            //如果数据缓存区的第一个字节不是0x55
            //就把缓冲区中的剩余数据向前移动一个字节
            if(s_ucWitDataBuff[0] != 0x55)
            {
                s_uiWitDataCnt--;
                memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                return ;
            }
            //当到达11个字节时，就会开始校验数据（其实s_uiWitDataCnt不会超过11的）
            //数据的格式为（共11字节），其中数据头规定为0x55
            //数据头 数据内容 数据1L 数据1H 数据2L 数据2H 数据3L 数据3H 数据4L 数据4H 校验和
            if(s_uiWitDataCnt >= 11)
            {
                //计算前十个数据的校验和并与第11个数据比较
                ucSum = __CaliSum(s_ucWitDataBuff, 10);
                if(ucSum != s_ucWitDataBuff[10])
                {
                    //如果校验失败，那么就把第一个字节丢弃，并且后续的字节向前移动
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                //把数据写入usData数组
                usData[0] = ((uint16_t)s_ucWitDataBuff[3] << 8) | (uint16_t)s_ucWitDataBuff[2];
                usData[1] = ((uint16_t)s_ucWitDataBuff[5] << 8) | (uint16_t)s_ucWitDataBuff[4];
                usData[2] = ((uint16_t)s_ucWitDataBuff[7] << 8) | (uint16_t)s_ucWitDataBuff[6];
                usData[3] = ((uint16_t)s_ucWitDataBuff[9] << 8) | (uint16_t)s_ucWitDataBuff[8];
                //s_ucWitDataBuff[1]里面是数据内容，0x52代表角速度，0x53代表角度
                CopeWitData(s_ucWitDataBuff[1], usData, 4);
                //清空计数器
                s_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_MODBUS:
            if(s_uiWitDataCnt > 2)
            {
                if(s_ucWitDataBuff[1] != FuncR)
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                if(s_uiWitDataCnt < (s_ucWitDataBuff[2] + 5))return ;
                usTemp = ((uint16_t)s_ucWitDataBuff[s_uiWitDataCnt-2] << 8) | s_ucWitDataBuff[s_uiWitDataCnt-1];
                usCRC16 = __CRC16(s_ucWitDataBuff, s_uiWitDataCnt-2);
                if(usTemp != usCRC16)
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                usTemp = s_ucWitDataBuff[2] >> 1;
                for(i = 0; i < usTemp; i++)
                {
                    sReg[i+s_uiReadRegIndex] = ((uint16_t)s_ucWitDataBuff[(i<<1)+3] << 8) | s_ucWitDataBuff[(i<<1)+4];
                }
                p_WitRegUpdateCbFunc(s_uiReadRegIndex, usTemp);
                s_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_CAN:
        case WIT_PROTOCOL_I2C:
        s_uiWitDataCnt = 0;
        break;
    }
    if(s_uiWitDataCnt == WIT_DATA_BUFF_SIZE)s_uiWitDataCnt = 0;
}

//往传感器的寄存器写数据的函数
//传入的是要写的寄存器地址和要发送给寄存器的16位数据
//內部本质上是把数据按协议要求修改后，通过Arduino的串口发送
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData)
{
    uint16_t usCRC;
    uint8_t ucBuff[8];
    //寄存器的地址越界则返回无效参数
    if(uiReg >= REGSIZE)return WIT_HAL_INVAL;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            //如果写函数没有注册，那么返回没有注册写入函数
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            //传感器的协议规定了发送数据的要求
            //第一个字节为0xFF = 11111111
            ucBuff[0] = 0xFF;
            //第二个字节为0xAA = 10101010
            ucBuff[1] = 0xAA;
            //第三个字节为寄存器的地址，相当于取低八位（更高位也都是0）
            ucBuff[2] = uiReg & 0xFF;
            //第四个字节是数据的低八位，同样是和11111111按位与
            ucBuff[3] = usData & 0xFF;
            //第五个字节是数据的高八位，将数据右移八位实现
            ucBuff[4] = usData >> 8;
            //把这五个字节向传感器发送，传感器接收到之后会自动处理
            p_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncW;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = usData >> 8;
            ucBuff[5] = usData & 0xff;
            usCRC = __CRC16(ucBuff, 6);
            ucBuff[6] = usCRC >> 8;
            ucBuff[7] = usCRC & 0xff;
            p_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(p_WitI2cWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = usData & 0xff;
            ucBuff[1] = usData >> 8;
			if(p_WitI2cWriteFunc(s_ucAddr << 1, uiReg, ucBuff, 2) != 1)
			{
				//printf("i2c write fail\r\n");
			}
        break;
	default: 
            return WIT_HAL_INVAL;        
    }
    //数据发送成功会返回0，但是不意味着传感器成功接收
    return WIT_HAL_OK;
}
//读取传感器某个寄存器的值
//实际过程是，Arduino向传感器发出请求，然后传感器把对应寄存器的数据发送过来
//函数是向传感器发送数据，发送要读取的寄存器地址，以及要读的数据个数（不超过4）
//然后数据的最终读取需要自己写函数读取
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum)
{
    uint16_t usTemp, i;
    uint8_t ucBuff[8];
    //检测要读取的寄存器是否为非法参数
    if((uiReg + uiReadNum) >= REGSIZE)return WIT_HAL_INVAL;
    //首先是读取请求的发送，告诉传感器要读取的寄存器的地址，以及从该寄存器开始要读几个数据
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            //如果一次性要读取超过四个数据，则也是非法参数
            if(uiReadNum > 4)return WIT_HAL_INVAL;
            //如果“向传感器发送请求的函数”没有注册，那么会返回缺少注册函数
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            //传感器的协议规定了发送数据的要求
            //第一个字节为0xFF = 11111111
            ucBuff[0] = 0xFF;
            //第二个字节为0xAA = 10101010
            ucBuff[1] = 0xAA;
            //第三个字节为寄存器的地址
            //0x27为READADDR，该寄存器存放要读取的寄存器的地址
            ucBuff[2] = 0x27;
            //第四个字节是寄存器地址的低八位
            ucBuff[3] = uiReg & 0xff;
            //第五个字节是寄存器地址的高八位
            ucBuff[4] = uiReg >> 8;
            //把这五个字节向传感器发送，传感器接收到之后会自动处理
            p_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if((usTemp + 5) > WIT_DATA_BUFF_SIZE)return WIT_HAL_NOMEM;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncR;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = uiReadNum >> 8;
            ucBuff[5] = uiReadNum & 0xff;
            usTemp = __CRC16(ucBuff, 6);
            ucBuff[6] = usTemp >> 8;
            ucBuff[7] = usTemp & 0xff;
            p_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(uiReadNum > 3)return WIT_HAL_INVAL;
            if(p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(p_WitI2cReadFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if(WIT_DATA_BUFF_SIZE < usTemp)return WIT_HAL_NOMEM;
            if(p_WitI2cReadFunc(s_ucAddr << 1, uiReg, s_ucWitDataBuff, usTemp) == 1)
            {
                if(p_WitRegUpdateCbFunc == NULL)return WIT_HAL_EMPTY;
                for(i = 0; i < uiReadNum; i++)
                {
                    sReg[i+uiReg] = ((uint16_t)s_ucWitDataBuff[(i<<1)+1] << 8) | s_ucWitDataBuff[i<<1];
                }
                p_WitRegUpdateCbFunc(uiReg, uiReadNum);
            }
			
            break;
		default: 
            return WIT_HAL_INVAL;
    }
    //更新全局变量，该变量存放要读取的寄存器的地址
    s_uiReadRegIndex = uiReg;
    return WIT_HAL_OK;
}



//Z轴角度(Yaw)归零
int32_t WitStartIYAWCali(void)
{
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	    return  WIT_HAL_ERROR;// unlock reg
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else;
	if(WitWriteReg(CALIYAW, 0x00) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else;
    if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
//设置Z轴角度
int32_t WitSetYAW(int32_t uiAngle)
{
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	    return  WIT_HAL_ERROR;// unlock reg
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else;
	if(WitWriteReg(CALIYAW, uiAngle) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else;
    if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
//设置传感器发送数据的速率，RRATE_50HZ传入这个就行
int32_t WitSetOutputRate(int32_t uiRate)
{	
	if(!CheckRange(uiRate,RRATE_02HZ,RRATE_NONE))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else;
	if(WitWriteReg(RRATE, uiRate) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else;
    if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
//设置波特率，WIT_BAUD_115200传入这个就行
int32_t WitSetUartBaud(int32_t uiBaudIndex)
{
	if(!CheckRange(uiBaudIndex,WIT_BAUD_4800,WIT_BAUD_921600))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else;
    if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
//保存设置
int32_t WitSave(void)
{
	if(WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
//这个一般不用改的，工作模式就用正常模式就行了
int32_t WitSetWORKMODE(void)
{
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	    return  WIT_HAL_ERROR;// unlock reg
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(WORKMODE, 0x00) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}


//开启休眠
int32_t WitStartSLEEPCali(void)
{
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	    return  WIT_HAL_ERROR;// unlock reg
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(SLEEP, 0x01) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
//设置频带宽度，串行通信用不上
int32_t WitSetBandwidth(int32_t uiBaudWidth)
{	
	if(!CheckRange(uiBaudWidth,BANDWIDTH_256HZ,BANDWIDTH_5HZ))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(BANDWIDTH, uiBaudWidth) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}


//用于查看发送值得范围是否符合（不怎么影响全局）
char CheckRange(short sTemp,short sMin,short sMax)
{
    if ((sTemp>=sMin)&&(sTemp<=sMax)) return 1;
    else return 0;
}


//I2C相关的可以不看
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func)
{
    if(!write_func)return WIT_HAL_INVAL;
    if(!read_func)return WIT_HAL_INVAL;
    p_WitI2cWriteFunc = write_func;
    p_WitI2cReadFunc = read_func;
    return WIT_HAL_OK;
}
//CAN通信相关的可以不看
int32_t WitCanWriteRegister(CanWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitCanWriteFunc = Write_func;
    return WIT_HAL_OK;
}
void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen)
{
	uint16_t usData[3];
    if(p_WitRegUpdateCbFunc == NULL)return ;
    if(ucLen < 8)return ;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_CAN:
            if(ucData[0] != 0x55)return ;
            usData[0] = ((uint16_t)ucData[3] << 8) | ucData[2];
            usData[1] = ((uint16_t)ucData[5] << 8) | ucData[4];
            usData[2] = ((uint16_t)ucData[7] << 8) | ucData[6];
            CopeWitData(ucData[1], usData, 3);
            break;
        case WIT_PROTOCOL_NORMAL:
        case WIT_PROTOCOL_MODBUS:
        case WIT_PROTOCOL_I2C:
            break;
    }
}
//改变CAN通信波特率
int32_t WitSetCanBaud(int32_t uiBaudIndex)
{
	if(!CheckRange(uiBaudIndex,CAN_BAUD_1000000,CAN_BAUD_3000))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

