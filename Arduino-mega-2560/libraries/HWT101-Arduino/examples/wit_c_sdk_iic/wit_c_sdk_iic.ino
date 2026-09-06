#include <Wire.h>
#include <REG.h>
#include <wit_c_sdk.h>

/*
 该例程是基于 MEGA 2560开发板的，如使用其他开发板需要自己做移植.，这个程序是基于jy901s开发的，也使用于101这个型号，101只有Z轴角速度Z轴角度数据。如需做其他修改需要根据101的协议去做开发修改
 JY901S           MEGA 2560
  VCC      <--->   5V/3.3V
  SCL      <--->   SCL(21)
  SDA      <--->   SDA(20)
  GND      <--->    GND
*/

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static char s_cDataUpdate = 0, s_cCmd = 0xff;

static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
static void Delayms(uint16_t ucMs);

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);
	Serial.begin(115200);
	WitInit(WIT_PROTOCOL_I2C, 0x50);
	WitI2cFuncRegister(IICwriteBytes, IICreadBytes);
	WitRegisterCallBack(CopeSensorData);
  WitDelayMsRegister(Delayms);
	Serial.print("\r\n********************** wit-motion IIC example  ************************\r\n");
	AutoScanSensor();
}
int i;
float fAcc[3], fGyro[3], fAngle[3];
void loop() {
    WitReadReg(AX, 12);
    delay(500);
    while (Serial.available()) 
    {
      CopeCmdData(Serial.read());
    }
		CmdProcess();
		if(s_cDataUpdate)
		{
			for(i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
			
			if(s_cDataUpdate & GYRO_UPDATE)
			{
				Serial.print("gyro	Z:");
				#
				Serial.print(fGyro[2], 1);
				Serial.print("\r\n");
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			if(s_cDataUpdate & ANGLE_UPDATE)
			{
				Serial.print("angleZ:");
				#
				Serial.print(fAngle[2], 3);
				Serial.print("\r\n");
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
			
      s_cDataUpdate = 0;
		}
}


void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}
}

static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
	int val;
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.endTransmission(false); //endTransmission but keep the connection active

    val = Wire.requestFrom(dev, length); //Ask for bytes, once done, bus is released by default

	if(val == 0)return 0;
    while(Wire.available() < length) //Hang out until we get the # of bytes we expect
    {
      if(Wire.getWireTimeoutFlag())
      {
        Wire.clearWireTimeoutFlag();
        return 0;
      }
    }

    for(int x = 0 ; x < length ; x++)    data[x] = Wire.read();   

    return 1;
}


static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.write(data, length);
    if(Wire.getWireTimeoutFlag())
    {
      Wire.clearWireTimeoutFlag();
      return 0;
    }
    Wire.endTransmission(); //Stop transmitting

    return 1; 
}

static void ShowHelp(void)
{
	Serial.print("\r\n************************	 WIT_SDK_DEMO	************************");
	Serial.print("\r\n************************          HELP           ************************\r\n");
	Serial.print("UART SEND:K\\r\\n   Z-axis zero setting.\r\n");  //101Z轴置零
  	Serial.print("UART SEND:A\\r\\n   AUTO  setting.\r\n");  //101自动获取零偏开始
  	Serial.print("UART SEND:P\\r\\n   AUTO  setting stop.\r\n");  //101停止自动获取零偏
	Serial.print("UART SEND:h\\r\\n   help.\r\n");
	Serial.print("******************************************************************************\r\n");
}

static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'K':  if (WitStartIYAWCali() != WIT_HAL_OK) Serial.print("\r\nSet IYAWCali Error\r\n"); //判断if括号里面的条件是否成立，成立就在PC端上打印出来
     			 break;
      		case 'A':  if (WitStartRKMODECali() != WIT_HAL_OK) Serial.print("\r\nSet RKMODECali Error\r\n"); //判断if括号里面的条件是否成立，成立就在PC端上打印出来
      			break;
   	               case 'P':  if (WitStopRKMODECali() != WIT_HAL_OK) Serial.print("\r\nSet RKMODECali Error\r\n"); //判断if括号里面的条件是否成立，成立就在PC端上打印出来
      			break;
		case 'h':	ShowHelp();
			break;
		default :return;
	}
	s_cCmd = 0xff;
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
         #
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
          #
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void Delayms(uint16_t ucMs)
{
  delay(ucMs);
}


static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < 0x7F; i++)
	{
		WitInit(WIT_PROTOCOL_I2C, i);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			delay(5);
			if(s_cDataUpdate != 0)
			{
				Serial.print("find 0x");
				Serial.print(i, HEX);
				Serial.print(" addr sensor\r\n");
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	Serial.print("can not find sensor\r\n");
	Serial.print("please check your connection\r\n");
}
