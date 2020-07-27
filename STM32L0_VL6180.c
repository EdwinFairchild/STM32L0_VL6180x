#include <stm32l0xx_ll_bus.h>
#include <stm32l0xx_ll_gpio.h>
#include <stm32l0xx_ll_utils.h>
#include <stm32l0xx_ll_i2c.h>
#include <stm32l0xx_ll_usart.h>


//---------------| CL |------------------------
#include  "CL_CONFIG.h"
#include "CL_printMsg.h"
#include "CL_delay.h"
#include "CL_systemClockUpdate.h"
//---------------| VL6180x |------------------------
#include "vl6180x_api.h"
#define def_i2c_time_out 500
 VL6180xDev_t myDev;
#define MyDev_Init(dev) (dev=0x52)
#define ALPHA (int)(0.85*(1<<16))
//---------------| Protoypes |------------------------
int  VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t  *buff, uint8_t len);
int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len);

//-----
static inline void MyDev_ShowLux(VL6180xDev_t dev, int lux);
void Sample_SimpleAls(void);
static inline void MyDev_ShowRange(VL6180xDev_t dev, int range, int duration);
void Sample_SimpleRanging(void);
void init_CL(void);
void init_i2c(void);
void i2c_read(uint8_t *data, uint8_t count);
void i2c_write(uint8_t *data, uint8_t len);
void blinkLed(uint8_t delay, uint8_t count);
void initLed(void);
void initUART(void);

#define theVL6180xDev   0x52    // what we use as "API device

//-------------------------------------------------------------------
int main(void)
{	
	//essentials
	init_CL();
	
	
	initLed();
	init_i2c();
	
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_LOW);
	
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
	delayMS(1000);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
	delayMS(1000);
	
	VL6180x_WaitDeviceBooted(myDev);
	VL6180x_InitData(myDev);

	CL_printMsg("VL6180 initialised\n");
	
	
	/*<><><>><><><><><><><><><><><><><><><><><><><><><><><><><><><><>*/
	
	VL6180xDev_t myDev;
	VL6180x_RangeData_t Range;
	int status;

	MyDev_Init(myDev);             // your code init device variable
	
	delayMS(1);            // your code sleep at least 1 msec
	VL6180x_InitData(myDev);
	VL6180x_Prepare(myDev);

	VL6180x_RangeClearInterrupt(myDev);   // make sure no interrupt is pending

/* kick off the first measurement */
		VL6180x_RangeStartSingleShot(myDev);
	for (;;)
	{
		
		
	
	
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3);
		delayMS(10);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);
		delayMS(10);
	}
	/*<><><>><><><><><><><><><><><><><><><><><><><><><><><><><><><><>*/
	
	
}//-------------------------------------------------------------------
int  VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t  *buff, uint8_t len)
{
	//int status;
	//status = HAL_I2C_Master_Transmit(&hi2c1, 0x52, buff, len, def_i2c_time_out);
	//return status;
	LL_I2C_SetTransferSize(I2C1, len);
	LL_I2C_SetSlaveAddr(I2C1, 0x52);
	LL_I2C_SetTransferRequest(I2C1,LL_I2C_REQUEST_WRITE);
	LL_I2C_GenerateStartCondition(I2C1);
	
	for (int i =  0; i < len; i++)
	{	
		while (!(I2C1->ISR & I2C_ISR_TXE)) ;
		LL_I2C_TransmitData8(I2C1, *buff++);		
	}
	
	return 0;
}//-------------------------------------------------------------------
int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len)
{
	//	int status;
	//	status = HAL_I2C_Master_Receive(&hi2c1, 0x52, buff, len, def_i2c_time_out);


		//return status;
	
	LL_I2C_SetTransferSize(I2C1, len);
	LL_I2C_SetSlaveAddr(I2C1, 0x52);
	LL_I2C_SetTransferRequest(I2C1,LL_I2C_REQUEST_READ);
	LL_I2C_GenerateStartCondition(I2C1);
	
	for (int i =  0; i < len; i++)
	{	
		while (!(I2C1->ISR & I2C_ISR_RXNE)) ;
		*buff++ = LL_I2C_ReceiveData8(I2C1);	
	}
	return 0;
}//-------------------------------------------------------------------
void Sample_FreeRunningRanging(void) {
	VL6180xDev_t myDev;
	VL6180x_RangeData_t Range;
	int status;

	MyDev_Init(myDev);            // your code init device variable

	delayMS(1000);           // your code sleep at least 1 msec
	VL6180x_InitData(myDev);
	VL6180x_Prepare(myDev);

	VL6180x_RangeClearInterrupt(myDev);  // make sure no interrupt is pending

	/* kick off the first measurement */
	VL6180x_RangeStartSingleShot(myDev);
	
	
	
//this part should get looped

		// TODO add your code anything in a loop way
		VL6180x_PollDelay(dev);  // simply  run default API poll delay that handle display in demo
		// check for range measure availability
		status = VL6180x_RangeGetMeasurementIfReady(myDev, &Range);
		if (status == 0) {         
			// Application must check Range.errorStatus before accessing the other data
			//    If Range.errorStatus is DataNotReady, application knows that it has to wait a bit before getting a new data
			//    If Range.errorStatus is 0, application knows it is a valid distance
			//    If Range.errorStatus is not 0, application knows that reported distance is invalid so may take some decisions depending on the errorStatus
			if(Range.errorStatus == DataNotReady)
			    ;
            
			if (Range.errorStatus == 0)
				MyDev_ShowRange(myDev, Range.range_mm, 0);  // your code display range in mm
			else
			   // MyDev_ShowErr(myDev, Range.errorStatus);  // your code display error code
			/* re-arm next measurement */ 
			VL6180x_RangeStartSingleShot(myDev);   
		}
		else {
			// it is an critical error
			//HandleError("critical error on VL6180x_RangeCheckAndGetMeasurement");
		}

 // your code to stop looping

	
}//-------------------------------------------------------------------
static inline void MyDev_ShowLux(VL6180xDev_t dev, int lux) {
	static char str[8];
	if (lux >= 1000)
		CL_printMsg( "l%3d \n", lux / 1000);
	else
		CL_printMsg( "l%3d \n ", lux);
	
}//-------------------------------------------------------------------
void Sample_SimpleAls(void) {
#if VL6180x_ALS_SUPPORT
	VL6180xDev_t myDev;
	VL6180x_AlsData_t Als;

	MyDev_Init(myDev);            // your code init device variable

	delayMS(1000);           // your code sleep at least 1 msec
	
	VL6180x_Prepare(myDev);
	do {
		VL6180x_AlsPollMeasurement(myDev, &Als);
		if (Als.errorStatus == 0)
			MyDev_ShowLux(myDev, Als.lux);  // your code display range in mm
		else
		    CL_printMsg("Error Code: %d\n", Als.errorStatus);  // your code display error code
	} while (1); // your code to stop looping
#endif
}//-------------------------------------------------------------------
static inline void MyDev_ShowRange(VL6180xDev_t dev, int range, int duration) 
{

	static int Range;  // must remain valid for

	Range = (Range * ALPHA + range * ((1 << 16) - ALPHA)) >> 16;
	CL_printMsg("r%3d\n", Range);


}//-------------------------------------------------------------------
void Sample_SimpleRanging(void) 
{
//this part only needs to be called once
	VL6180x_RangeData_t Range;
	MyDev_Init(myDev);            // your code init device variable
	//VL6180x_InitData(myDev);
	VL6180x_Prepare(myDev);

// this part can  loop	
	VL6180x_RangePollMeasurement(myDev, &Range);
	if (Range.errorStatus == 0)
	{	
		MyDev_ShowRange(myDev, Range.range_mm, 0);  // your code display range in mm
	}
	

}//-------------------------------------------------------------------
void init_CL(void)
{
	setClockTo32Mhz();	
	CL_delay_init();
	CL_printMsg_init_Default(false);	
	CL_printMsg("CL Initialised\n");
}//-------------------------------------------------------------------
void init_i2c(void)
{
	//-----------clocks
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1); 
	
	//  [PA9]  = [D1] --> I2C1_SCL 
	//  [PA10] = [D0] --> I2C1_SDA 
	LL_GPIO_InitTypeDef gpio;
	
	gpio.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	//gpio.Pull = LL_GPIO_PULL_UP;
	gpio.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &gpio);

	// ----------I2C1
	LL_I2C_InitTypeDef i2c;
	i2c.Timing = (uint32_t)0x00300F38;
	i2c.TypeAcknowledge = LL_I2C_ACK;

	LL_I2C_StructInit(&i2c);
	LL_I2C_EnableAutoEndMode(I2C1);	
	LL_I2C_Init(I2C1, &i2c);	
	
	LL_I2C_Enable(I2C1);
	
}//-------------------------------------------------------------------
void i2c_read(uint8_t *data, uint8_t count)
{
	LL_I2C_SetTransferSize(I2C1, count);
	LL_I2C_SetSlaveAddr(I2C1, 0x52);
	LL_I2C_GenerateStartCondition(I2C1);
	
	for (int i =  0; i < count; i++)
	{	
		while (!(I2C1->ISR & I2C_ISR_RXNE)) ;
		*data++ = LL_I2C_ReceiveData8(I2C1);	
	}
}//-------------------------------------------------------------------
void i2c_write(uint8_t *data, uint8_t len)
{	
	LL_I2C_SetTransferSize(I2C1, len);
	LL_I2C_SetSlaveAddr(I2C1, 0x52);
	LL_I2C_GenerateStartCondition(I2C1);
	
	for (int i =  0; i < len; i++)
	{	
		while (!(I2C1->ISR & I2C_ISR_TXE)) ;
		LL_I2C_TransmitData8(I2C1, *data++);		
	}
	
	//autoend enabled
	//LL_I2C_GenerateStopCondition(I2C1);
}//-------------------------------------------------------------------
void initUART(void)
{
	/*  For use with Virtual Com port of the L0 Nucleo
	 *  [PA2] [USART_2_TX : AF 4 ]<--->[PA15] [USAR_2__RX : AF 4 ]
	 *	  
	*/
	/*
	// Clocks
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	
	// GPIO 
	LL_GPIO_InitTypeDef uartPins;
	LL_GPIO_StructInit(&uartPins);	
	
	uartPins.Pin		= LL_GPIO_PIN_2;//| LL_GPIO_PIN_15; 
	uartPins.Mode		= LL_GPIO_MODE_ALTERNATE;
	uartPins.Alternate	= LL_GPIO_AF_4;		
	LL_GPIO_Init(GPIOA, &uartPins);	
	
	// USART 2
	LL_USART_InitTypeDef usart2;
	LL_USART_StructInit(&usart2); //defualt values work
	
	usart2.BaudRate = 115200U;	
	
	LL_USART_Init(USART2, &usart2);
	LL_USART_Enable(USART2);
	*/
	
	//enable  clock  needed for uart gpio aswell as uart itself
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	// PA2 and PA15 to Alternate Function Mode
	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE2_0))
			| (GPIO_MODER_MODE2_1);

	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE15_0))
			| (GPIO_MODER_MODE15_1);

	//Select the specific Alternate function
	GPIOA->AFR[0] |= 4 << GPIO_AFRL_AFSEL2_Pos;
	GPIOA->AFR[1] |= 4 << GPIO_AFRH_AFSEL15_Pos;

	// Baudrate = clk_Frq / BRR ===>  32Mhz / 9600 = 0xD05
	USART2->BRR = 0xD05;   //160000 / 96;
	// Enable RX_NE interrupt and TXE interrupt, enable UART, RECEIVE , TRANSMIT COMPLETE
	USART2->CR1 = USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE	| USART_CR1_TCIE;

}
void blinkLed(uint8_t delay, uint8_t count)
{
	for (int i = 0; i < count; i++)
	{		
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);
		LL_mDelay(1000);
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3);
		LL_mDelay(1000);
	}
}//-------------------------------------------------------------------
void initLed(void)
{
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_LOW);
	
}//-------------------------------------------------------------------
