#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "sysdef.h"
#include "sockets.h"
#include "flash_if.h"
#include <string.h>

#define JNZS_DEBUG printf
#define JNZS_INFO printf
#define SAVESTART (0x01020304)
#define SAVEEND   (0x04030201)

extern osThreadId dispTaskHandle;
extern osThreadId UARTCMDTaskHandle;
extern osThreadId ETHCMDTaskHandle;
extern osThreadId RWSensorIOTaskHandle;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

unsigned long ETHstack;
unsigned long sensorstack;
unsigned long GUIstack;
unsigned long Touchstack;
unsigned long UARTstack;
typedef struct sensor_data
{
	float WaterPressBef;	
	float WaterPressAft;	
	float OilPress;	
	float OilTemp;	
	float OilPos;	
	float OilVel;	
	int InStat;
	int OutStat;	
	int DACVal1;	
	int DACVal2;	
}SENSORDATA;
typedef struct Ctrlinfo
{
	int PN;	
  int SN;	
  int ManuData;	
  int HWVer;
  int SWVer;	
	float adcscale[8];  //convert 0~4095 to sensor's real range
	float adcoffset[8]; //convert 0~4095 to sensor's real range
}CTRLINFO;
typedef struct saveInfo
{
	int start;
	float RANGE[8][2];
	float temper[2];  //below temper 0: close fan;above temper 1: start fan;
	float reserve[2];
	int end;
}SAVEINFO;



typedef struct cmdheader
{
	char mode;
	char bytenum;
	short cmd;
}CMDHEADER;

extern char guartbuf[MAXCOMLEN];
char gethbuf[MAXCOMLEN];


SENSORDATA gsensor={0};
CTRLINFO gctrlinfo={0};
SAVEINFO gsaveinfo={0};
RcvS gRev;
RcvS gRev3;

u16 Get_Adc(ADC_HandleTypeDef adcx, u32 ch)   
{      
    
	  ADC_ChannelConfTypeDef ADC1_ChanConf;
    
    ADC1_ChanConf.Channel=ch;                                  
    ADC1_ChanConf.Rank=1;                                       
    //ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_480CYCLES;        
    //ADC1_ChanConf.Offset=0;                 
    HAL_ADC_ConfigChannel(&adcx,&ADC1_ChanConf);        
	
	  HAL_ADC_Start(&adcx);                               
    HAL_ADC_PollForConversion(&adcx,10);               
   
	return (u16)HAL_ADC_GetValue(&adcx);	            
}

void Get_Adc2(u32 ch1, u32 ch2, uint32_t *data1, uint32_t *data2)   
{      
    
	  ADC_ChannelConfTypeDef ADC1_ChanConf;
    
    ADC1_ChanConf.Channel=ch1;                                                                       
    ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_480CYCLES;                      
    HAL_ADC_ConfigChannel(&hadc2,&ADC1_ChanConf);        
	  HAL_ADC_Start(&hadc2);
	
	  ADC1_ChanConf.Channel=ch2;                                                                                           
    HAL_ADC_ConfigChannel(&hadc1,&ADC1_ChanConf);        
	  HAL_ADC_Start(&hadc1);

	
    HAL_ADC_PollForConversion(&hadc2,10);  
    HAL_ADC_PollForConversion(&hadc1,10); 	
    
	  *data1 = HAL_ADC_GetValue(&hadc1);
	  *data2 = HAL_ADC_GetValue(&hadc2);
	return;	            
}

u32 Get_Adc_Average(ADC_HandleTypeDef adcx, u32 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(adcx, ch);
		//osDelay(5);
	}
	return temp_val/times;
} 

void Get_Adc_Average2(u32 ch1, u32 ch2, u32 *data1, u32 *data2, u32 times)
{
	u32 temp_val1=0,cur1;
	u32 temp_val2=0,cur2;
	u32 t;
	for(t=0;t<times;t++)
	{
		Get_Adc2(ch1,ch2,&cur1, &cur2);
		temp_val1+=cur1;
		temp_val2+=cur2;
	}
	*data1 = temp_val1/times;
	*data2 = temp_val2/times;
	return;
} 


short Get_Temprate(void)
{
	u32 adcx;
	short result;
 	double temperate;
	adcx=Get_Adc_Average(hadc1, ADC_CHANNEL_TEMPSENSOR,10);
	temperate=(float)adcx*(3.3/4096);		
	temperate=(temperate-0.76)/0.0025 + 25; 
	result=temperate*=100;				
	return result;
}


void appdatainit()
{
	int i;
	int *ptemp = (int *)USER_FLASH_FIRST_PAGE_ADDRESS;
	//sensor range,  used for calculate transform coefficient
  float initRANGE[8][2] = { { 0.0, 30.0 }, \
												{0.0, 30.0}, \
												{0.0, 6.0}, \
												{-50, 100.0}, \
												{0.0, 4096.0}, \
												{0.6, 6.0}, \
												{0.0, 4096.0}, \
												{0.0, 4096.0} };
	float x1 = 0.6 * 4096 / 3.3;
	float x2 = 3 * 4096 / 3.3;										
	
  //read configuration	from flash											
	if(*ptemp == SAVESTART)
	{
		printf("read cfg form flash!..........\n");
		memcpy((void *)&gsaveinfo, (void*)ptemp, sizeof(gsaveinfo));
	}
	//flash is empty, init...
	else
	{
		printf("init save info..........\n");
		gsaveinfo.start = SAVESTART;
		gsaveinfo.end = SAVEEND;
		memcpy((void *)gsaveinfo.RANGE, (void *)initRANGE, 8*2*4);
		gsaveinfo.temper[0]=20.0;
		gsaveinfo.temper[1]=30.0;
	}
	
	for(i=0;i<8;i++)
	{
			gctrlinfo.adcscale[i] = (gsaveinfo.RANGE[i][1] - gsaveinfo.RANGE[i][0]) / (x2 - x1);
			gctrlinfo.adcoffset[i] = (gsaveinfo.RANGE[i][0] * x2 - (gsaveinfo.RANGE[i][1])*x1) / (x2 - x1);
	}
	gctrlinfo.HWVer = 0x100;
	gctrlinfo.PN = 0x100;
	gctrlinfo.SWVer=0x201705;
	
  gRev.sr1 = 0;
	gRev.sr2 = 0;
	gRev.cnt = 0;
	
	gRev3.sr1 = 0;
	gRev3.sr2 = 0;
	gRev3.cnt = 0;
	
	gsensor.DACVal1 = 1024;
	gsensor.DACVal2 = 1024;
}
extern osSemaphoreId gDataSemHandle;

void sem_memcpy(void *dst, void *src, int cnt)
{
	osSemaphoreWait(gDataSemHandle, osWaitForever);
	memcpy(dst, src, cnt);
	osSemaphoreRelease(gDataSemHandle);
}

void saveCfg()
{
		unsigned int flashaddr = USER_FLASH_FIRST_PAGE_ADDRESS;
		__disable_irq(); //close irq
	  osThreadSuspendAll();  //close thread dispatcher
	
		FLASH_If_Init();
		FLASH_If_Erase(USER_FLASH_FIRST_PAGE_ADDRESS);
		FLASH_If_Write(&flashaddr, (unsigned int *)&gsaveinfo, sizeof(gsaveinfo)/4);
		__enable_irq();
	  osThreadResumeAll();
}

/*return:
 1: process ok
 0: com buf error
*/
int processCom(char *rec, char *send, int* sendcnt)
{
	int ret=1;
	CMDHEADER *pheader;
	CMDHEADER *pheader_s;
	pheader = (CMDHEADER *)rec;
  pheader_s = (CMDHEADER *)send;
	
	pheader_s->mode = pheader->mode;
	pheader_s->cmd = pheader->cmd;
	if(pheader->mode == 'G')
	{		
		if(pheader->cmd<10)
		{
			pheader_s->bytenum = 1;
			sem_memcpy(send+4, (void *)(((int *)&gsensor)+pheader->cmd), 4);
		
		}
		else if(pheader->cmd == 100)
		{
			pheader_s->bytenum = 10;
			sem_memcpy(send+4, (void *)&gsensor, 4*10);
		}
  }
	else if(pheader->mode == 'S')
	{
		if((pheader->cmd == 8) || (pheader->cmd == 9) || (pheader->cmd == 7))
		{
			pheader_s->bytenum = 1;
			sem_memcpy((void *)(((int *)&gsensor)+pheader->cmd), rec+4, 4);
		}
		else if((pheader->cmd == 200) && (pheader->bytenum == 3 ))
		{
			pheader_s->bytenum = 3;
			sem_memcpy((void *)(((int *)&gsensor)+7), rec+4, 4*3);
		}
		else if(pheader->cmd == 300)
		{
			saveCfg();
		}
	}
	else
	{
		ret = 0;
	}
	send[4+4*pheader_s->bytenum] = 0x0D;
	send[4+4*pheader_s->bytenum + 1] = 0x0A;
	*sendcnt = 6 + 4 * pheader_s->bytenum;
	return ret;
}
#if 0
/* USER CODE END 4 */
extern float adc1val;
extern short Temperate;
/* StartETHCMDTask function */
void StartETHCMDTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
	
	
	//unsigned int adcx;
	//float temp;
	struct sockaddr_in my_addr; 
	unsigned short port = 8086;
	int sockfd;
	
	sockfd = socket(AF_INET, SOCK_STREAM, 0);   
  if(sockfd < 0)  
  {  
			printf("create socket error\n");
  }  
	
 
  //bzero(&my_addr, sizeof(my_addr));      
  my_addr.sin_family = AF_INET;   
  my_addr.sin_port   = htons(port);  
  my_addr.sin_addr.s_addr = htonl(INADDR_ANY);   
      
  
  int err_log = bind(sockfd, (struct sockaddr*)&my_addr, sizeof(my_addr));  
  if( err_log != 0)  
  {  
        printf("binding err\n");  
        closesocket(sockfd);         
  } 
	
	err_log = listen(sockfd, 1); 
  if(err_log != 0)  
  {  
        printf("listen error\n");  
        closesocket(sockfd);          
  }
 
  struct sockaddr_in client_addr;          
  char cli_ip[INET_ADDRSTRLEN] = "";       
  socklen_t cliaddr_len = sizeof(client_addr);         
  int connfd; 
  int flag = 1;
  int result; 	
	int rcvcnt;
	int sendcnt,sendout;
	char replybuf[MAXCOMLEN];
	CMDHEADER *pheader;
	pheader = (CMDHEADER *)gethbuf;
  /* Infinite loop */
  for(;;)
  {
     connfd = accept(sockfd, (struct sockaddr*)&client_addr, &cliaddr_len);         
     if(connfd < 0)  
     {  
         JNZS_DEBUG("accept error!!!");  
         continue;  
     }  
     result = setsockopt(connfd,            /* socket affected */
                                 IPPROTO_TCP,     /* set option at TCP level */
                                 TCP_NODELAY,     /* name of option */
                                 (char *) &flag,  /* the cast is historical
                                                         cruft */
                                 sizeof(int));    /* length of option value */

     if (result < 0)
     {
            JNZS_INFO("TCP_NODELAY  SET Error: %d\n", result);
     }
         
     inet_ntop(AF_INET, &client_addr.sin_addr, cli_ip, INET_ADDRSTRLEN);  
     JNZS_DEBUG("client ip=%s,port=%d\n", cli_ip,ntohs(client_addr.sin_port));  
          

     //Maybe a rec buf contain 2 com cmds,so read the cmd one be one
     while( (rcvcnt = recv(connfd, gethbuf, sizeof(CMDHEADER), 0)) > 0 ) //first read header
     {   
     
        if(rcvcnt == sizeof(CMDHEADER))
        {       
						rcvcnt = recv(connfd, gethbuf+sizeof(CMDHEADER), pheader->bytenum*4 + 2, 0);
						if((rcvcnt == pheader->bytenum*4 + 2) && (gethbuf[sizeof(CMDHEADER) + rcvcnt - 2] == 0x0D)) 
						{
									if(processCom( gethbuf, replybuf, &sendcnt) == 1)
									{
												 sendout = send(connfd, replybuf, sendcnt, 0);
												 JNZS_INFO("ETH: send buf len %d\n", sendout);
									}
									if( sendcnt < 0)
									 {
												JNZS_DEBUG("send err!\n");
												break;
									 }
						}
						else
						{
									 JNZS_DEBUG("recv char err!\n");
									 recv(connfd, gethbuf, MAXCOMLEN, 0);
						}
         }
         else
         {
              JNZS_DEBUG("recv not enough !\n");
              break;
         }
     }           
     closesocket(connfd);   
     JNZS_DEBUG("client closed!\n");  
		//osDelay(1000);
  }
  /* USER CODE END 5 */ 
}
#endif

extern UART_HandleTypeDef huart1;
extern osSemaphoreId guartSemHandle;
void StartUARTCMDTask(void const * argument)
{
	char replybuf[MAXCOMLEN];
	int sendcnt;
	memset(replybuf, 0, MAXCOMLEN);
	printf("UARTCMDTask task begin CMDHEADER=%d\n",sizeof(CMDHEADER));
	

	for(;;)
	{
		osSemaphoreWait(guartSemHandle, osWaitForever);
		if(processCom(guartbuf,replybuf, &sendcnt) == 1)
		{
			//HAL_UART_Transmit(&huart1, gRev.buf, gRev.cnt,1000);
			HAL_UART_Transmit_DMA(&huart1, (unsigned char*)replybuf, sendcnt);
		}
		else
		{
			printf("UARTCMDTask exec error\n");
		}
	}
}
/*4 to 1*/
void setADCMux(int code)
{
	GPIO_PinState temp = (GPIO_PinState)(code&0x1);
	HAL_GPIO_WritePin(GPIOB,CD4052A_Pin,temp); 
	temp = (GPIO_PinState)((code>>1)&0x1);
	HAL_GPIO_WritePin(GPIOB,CD4052B_Pin,temp);
}

int getDI()
{
	int ret=0;
	ret = HAL_GPIO_ReadPin(GPIN8_GPIO_Port, GPIN8_Pin);
	ret = ((ret << 1) &0xFE) | (HAL_GPIO_ReadPin(GPIN7_GPIO_Port, GPIN7_Pin) & 0x1);
	ret = ((ret << 1) &0xFE) | (HAL_GPIO_ReadPin(GPIN6_GPIO_Port, GPIN6_Pin) & 0x1);
	ret = ((ret << 1) &0xFE) | (HAL_GPIO_ReadPin(GPIN5_GPIO_Port, GPIN5_Pin) & 0x1);
	ret = ((ret << 1) &0xFE)  | (HAL_GPIO_ReadPin(GPIN4_GPIO_Port, GPIN4_Pin) & 0x1);
	ret = ((ret << 1) &0xFE)  | (HAL_GPIO_ReadPin(GPIN3_GPIO_Port, GPIN3_Pin) & 0x1);
	ret = ((ret << 1) &0xFE)  | (HAL_GPIO_ReadPin(GPIN2_GPIO_Port, GPIN2_Pin) & 0x1);
	ret = ((ret << 1) &0xFE)  | (HAL_GPIO_ReadPin(GPIN1_GPIO_Port, GPIN1_Pin) & 0x1);
	return ret;
}


void setDO(int dout)
{
	return;
}
extern DAC_HandleTypeDef hdac;
void DACout(int dac1, int dac2)
{
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,dac1);
	TIM4->CCR1=dac2; //TIM4 CH1, dac2:0~4095
	return;
}
int muxdelay = 1;
int avgcnt = 512;
/* USER CODE BEGIN 4 */
void StartRWSensorIOTask(void const * argument)
{
	printf("RWSensorIO task begin\n");
	int i;
	float fADCval[8];
	unsigned int adctemp1,adctemp2;
	int IOin;
	int IOout;
	int DACval[2];
	for(;;)
  {
		for(i=0; i<4;i++)
		{
			setADCMux(i);
			osDelay(muxdelay);	
			//adctemp = Get_Adc_Average(hadc1, ADC_CHANNEL_0, 40);  
			Get_Adc_Average2(ADC_CHANNEL_0, ADC_CHANNEL_9, &adctemp1, &adctemp2, avgcnt);
			fADCval[i] = adctemp1 * gctrlinfo.adcscale[i] + gctrlinfo.adcoffset[i];
			//adctemp = Get_Adc_Average(hadc2, ADC_CHANNEL_9, 40);  
			fADCval[i+4] = adctemp2 * gctrlinfo.adcscale[i+4] + gctrlinfo.adcoffset[i+4];
		}
		IOin = getDI();
		
		
		//sem_memcpy((void*)&gsensor,(void*)fADCval, 6*4);
		
		osSemaphoreWait(gDataSemHandle, osWaitForever);
		gsensor.WaterPressBef = fADCval[0];
		gsensor.WaterPressAft = fADCval[1];
		gsensor.OilPress = fADCval[2];
		gsensor.OilTemp = fADCval[3];
		gsensor.OilPos = fADCval[4];
		gsensor.OilVel = fADCval[5];
		gsensor.InStat = IOin;
		IOout = gsensor.OutStat;
		DACval[0] = gsensor.DACVal1;
		DACval[1] = gsensor.DACVal2;
		osSemaphoreRelease(gDataSemHandle);
		setDO(IOout);
		DACout(DACval[0], DACval[1]);	  
		osDelay(100);	
	}
}

extern UART_HandleTypeDef huart3;
unsigned int CMDDELAY=1;
//Notice: HMI UART LCD  Baudrate=115200. 
//HAL_UART_Transmit para Timeout must be larger than Tx Size.
void updateDisp()
{
	char temp[64];
	
	osDelay(CMDDELAY);
	sprintf(temp, "page0.A0.txt=\"%4.2f\"%c%c%c", gsensor.WaterPressBef, 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
	
	osDelay(CMDDELAY);
	sprintf(temp, "page0.A1.txt=\"%4.2f\"%c%c%c", gsensor.WaterPressAft, 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
	
	osDelay(CMDDELAY);
	sprintf(temp, "page0.A2.txt=\"%4.2f\"%c%c%c", gsensor.OilPress, 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
	
	osDelay(CMDDELAY);
	sprintf(temp, "page0.A3.txt=\"%4.1f\"%c%c%c", gsensor.OilTemp, 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
	
	osDelay(CMDDELAY);
	sprintf(temp, "page0.A4.txt=\"%4.1f\"%c%c%c", gsensor.OilPos, 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
	
	osDelay(CMDDELAY);
	sprintf(temp, "page0.A5.txt=\"%4.3f\"%c%c%c", gsensor.OilVel, 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);

	osDelay(CMDDELAY);
	sprintf(temp, "page0.C0.txt=\"%d\"%c%c%c", ((gsensor.InStat>>3) & 0x1), 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
	
	osDelay(CMDDELAY);
	sprintf(temp, "page0.C1.txt=\"%d\"%c%c%c", ((gsensor.InStat>>4) & 0x1), 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
		
}
void initParaDisp()
{
	char temp[64];
	int i,j;

	for(i=0;i<6;i++)
	{
		for(j=0;j<2;j++)
		{
			//i=0,j=0 => para.T201.txt=gsaveinfo.RANGE[0][0]
			osDelay(CMDDELAY);
			sprintf(temp, "para.T2%c%c.txt=\"%4.3f\"%c%c%c", i+0x30, j+1+0x30, gsaveinfo.RANGE[i][j], 0xff, 0xff, 0xff);
			HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
		}
	}
	osDelay(CMDDELAY);
	sprintf(temp, "para.T261.txt=\"%3.1f\"%c%c%c", gsaveinfo.temper[0], 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
	osDelay(CMDDELAY);
	sprintf(temp, "para.T262.txt=\"%3.1f\"%c%c%c", gsaveinfo.temper[1], 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
}

#define INITU3RCV {gRev3.sr1 = 0; gRev3.cnt = 0;\
									__HAL_UART_CLEAR_OREFLAG(&huart3);}/*\
									__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);}*/
#define WAITU3RCV {int to=0; do{osDelay(15);to++;} while((gRev3.sr1 == 0) &&(to<10));/*__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);*/} 

//buf format: 0x71 0xd2 0x04 0x00 0x00 0xff 0xff 0xff =>return 0x4d2
int getHMIint(unsigned char *buf, int len)
{
	int ret;
	if((len<4) ||(buf[0] != 0x71))
	{
		return -1;
	}
	memcpy((void *)&ret, (void *)(buf+1), 4);
	return ret;
}
//buf format: 0x71 0x30 0x2e 0x31 0x00 0xff 0xff 0xff   =>return 0.1
float getHMIfloat(unsigned char *buf, int len)
{
	float ret;
	
	if((len<4) ||(buf[0] != 0x70))
	{
		return -1;
	}
  sscanf((char *)(buf+1), "%f", &ret);
	return ret;
}
void toSave()
{
	char temp[64];
	int i,j;
	osDelay(CMDDELAY);
	INITU3RCV;
	sprintf(temp, "get sys1%c%c%c", 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
	WAITU3RCV;
	if(getHMIint(gRev3.buf, gRev3.cnt) == 1234) //sys1=1234 means user change para,need to save
	{		
			for(i=0;i<6;i++)
			{
				for(j=0;j<2;j++)
				{
					osDelay(CMDDELAY);
					INITU3RCV;
					//i=0,j=0 => get para.T201.txt
					sprintf(temp, "get para.T2%c%c.txt%c%c%c", i+0x30, j+1+0x30, 0xff, 0xff, 0xff);
					HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
					WAITU3RCV;
					gsaveinfo.RANGE[i][j] = getHMIfloat(gRev3.buf, gRev3.cnt);
				}
			}
			for(j=0;j<2;j++)
			{
				osDelay(CMDDELAY);
				INITU3RCV;
				sprintf(temp, "get para.T26%c.txt%c%c%c", j+1+0x30, 0xff, 0xff, 0xff);
				HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
				WAITU3RCV;
				gsaveinfo.temper[j] = getHMIfloat(gRev3.buf, gRev3.cnt);
			}
			osDelay(CMDDELAY);
			sprintf(temp, "sys1=0%c%c%c", 0xff, 0xff, 0xff);
			HAL_UART_Transmit(&huart3, (unsigned char*)temp, strlen(temp), 50);
			saveCfg();
			appdatainit();
	}
	return;
}

void StartdispTask(void const * argument)
{
	printf("Disp task begin.....\n");
	osDelay(500); //wait LCD start
	initParaDisp();
	for(;;)
  {
		
    HAL_GPIO_WritePin(GPIOB,LED1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,LED0_Pin,GPIO_PIN_SET);   
    osDelay(400);		
    //IWDG_Feed();		
    HAL_GPIO_WritePin(GPIOB,LED1_Pin,GPIO_PIN_SET);   
    HAL_GPIO_WritePin(GPIOB,LED0_Pin,GPIO_PIN_RESET); 
    osDelay(400); 
    updateDisp();
		toSave();
		
		//test OS Thread stack use, can be deleted 
		//ETHstack	= uxTaskGetStackHighWaterMark(ETHCMDTaskHandle);
		//sensorstack	= uxTaskGetStackHighWaterMark(RWSensorIOTaskHandle);
		//Touchstack	= uxTaskGetStackHighWaterMark(dispTaskHandle);
		//UARTstack	= uxTaskGetStackHighWaterMark(UARTCMDTaskHandle);
	
	}
		
}

