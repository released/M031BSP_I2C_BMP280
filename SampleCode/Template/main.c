/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "NuMicro.h"

#include	"project_config.h"
#include "i2c_driver.h"

//#include "bmp280.h"
#include "bmp2.h"
#include "common.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;

struct bmp2_dev dev;
struct bmp2_config conf;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void delay(uint16_t dly)
{
/*
	delay(100) : 14.84 us
	delay(200) : 29.37 us
	delay(300) : 43.97 us
	delay(400) : 58.5 us	
	delay(500) : 73.13 us	
	
	delay(1500) : 0.218 ms (218 us)
	delay(2000) : 0.291 ms (291 us)	
*/

	while( dly--);
}

void delay_us(uint32_t us)
{
	TIMER_Delay(TIMER0, us);
}

void delay_ms(uint32_t ms)
{
	TIMER_Delay(TIMER0, 1000*ms);
}

int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	#if defined (ENABLE_I2C_POLLING_DISCRETE)
	uint8_t i = 0;	
	uint8_t tmp = 0;	
	I2C_T *i2c = MASTER_I2C;	
	
	I2C_START(i2c);                    											//Start
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, (i2c_addr << 1) | I2C_WR );        						//send slave address
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	// refer to BMP280 datasheet , Figure 7: I2C multiple byte write (not auto-incremented)
	for (i = 0; i < length; i++)
	{
		I2C_SET_DATA(i2c, reg_addr + i);        									//send index
		I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
		I2C_WAIT_READY(i2c);

		tmp = reg_data[i];
		I2C_SET_DATA(i2c, tmp);            										//send Data
		I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
		I2C_WAIT_READY(i2c);
	}

	I2C_STOP(i2c);																//Stop
	#else
	I2Cx_WriteMultiToSlaveIRQ(i2c_addr , reg_addr , reg_data  , length);
	#endif

	return BMP2_INTF_RET_SUCCESS ;

    /* Implement the I2C write routine according to the target machine. */
//    return -1;
}

int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	#if defined (ENABLE_I2C_POLLING_DISCRETE)
	uint8_t i = 0;	
	uint8_t tmp = 0;
	I2C_T *i2c = MASTER_I2C;

	I2C_START(i2c);                         										//Start
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, (i2c_addr << 1) | I2C_WR );           						//send slave address+W
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, reg_addr);             									//send index
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

//	I2C_STOP(i2c);																//Stop
	////////////////////////////////////////////
//	I2C_START(i2c);                         									//Start
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_STA_SI);
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, (i2c_addr << 1) | I2C_RD );    							//send slave address+R
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	// refer to BMP280 datasheet , Figure 8: I2C multiple byte read 
	for (i = 0; i < length; i++)
	{
		if (i == (length -1))														//last byte : NACK
		{
//			I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO);
			I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
		}
		else			// ACK
		{
			I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
		}
		I2C_WAIT_READY(i2c);
		tmp = I2C_GET_DATA(i2c);           				//read data
		reg_data[i]=tmp;
	}

//	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
	
	#else
	I2Cx_ReadMultiFromSlaveIRQ(i2c_addr , reg_addr , reg_data  , length);
	#endif
	
	return BMP2_INTF_RET_SUCCESS ;

    /* Implement the I2C read routine according to the target machine. */
//    return -1;
}

int8_t bmp2_get_data(uint32_t period, struct bmp2_config *conf, struct bmp2_dev *dev)
{
    int8_t rslt = BMP2_E_NULL_PTR;
//    int8_t idx = 1;
    struct bmp2_status status;
    struct bmp2_data comp_data;
//	uint8_t temp[BMP2_P_T_LEN];
//	uint8_t i = 0;

//    printf("Measurement delay : %lu us\n", (long unsigned int)period);

//    while (idx <= 50)
    {
        rslt = bmp2_get_status(&status, dev);
        bmp2_error_codes_print_result("bmp2_get_status", rslt);

        if (status.measuring == BMP2_MEAS_DONE)
        {
            /* Delay between measurements */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bmp2_get_sensor_data(&comp_data, dev);
            bmp2_error_codes_print_result("bmp2_get_sensor_data", rslt);

            #ifdef BMP2_64BIT_COMPENSATION
            comp_data.pressure = comp_data.pressure / 256;
            #endif

			#if 1
			
            #ifdef BMP2_DOUBLE_COMPENSATION
            printf("Temperature: %.4lf deg C , Pressure: %.4lf Pa , %.4lf hPa\n",
                   comp_data.temperature,
                   comp_data.pressure,
                   comp_data.pressure/100);
            #else
            printf("Temperature: %ld deg C	Pressure: %lu Pa\n", comp_data.temperature,
                   comp_data.pressure);
            #endif

			#else
//			bmp2_get_regs(BMP2_REG_PRES_MSB, temp, BMP2_P_T_LEN, dev);
			i2c_reg_read(BMP2_I2C_ADDR_PRIM , BMP2_REG_PRES_MSB , (uint8_t *)temp, BMP2_P_T_LEN);
			for (i = 0; i <BMP2_P_T_LEN ; i++)
			{
				printf("[%d] : 0x%2X , " ,i ,temp[i]);
			}
			printf("\r\n");
			#endif

//            idx++;
        }
    }

    return rslt;
}

void sensor_polling(void)
{
    int8_t rslt;
	uint32_t meas_time;

    /* Calculate measurement time in microseconds */
    rslt = bmp2_compute_meas_time(&meas_time, &conf, &dev);
    bmp2_error_codes_print_result("bmp2_compute_meas_time", rslt);
	
    /* Read pressure and temperature data */
    rslt = bmp2_get_data(meas_time, &conf, &dev);
    bmp2_error_codes_print_result("get_data", rslt);
}

void sensor_API_Init(void)
{
    int8_t rslt;

    /* Interface selection is to be updated as parameter
     * For I2C :  BMP2_I2C_INTF
     * For SPI :  BMP2_SPI_INTF
     */
    rslt = bmp2_interface_selection(&dev, BMP2_I2C_INTF);
    bmp2_error_codes_print_result("bmp2_interface_selection", rslt);

    rslt = bmp2_init(&dev);
    bmp2_error_codes_print_result("bmp2_init", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    rslt = bmp2_get_config(&conf, &dev);
    bmp2_error_codes_print_result("bmp2_get_config", rslt);

    /* Configuring the over-sampling mode, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP2_FILTER_OFF;

    /* Over-sampling mode is set as high resolution i.e., os_pres = 8x and os_temp = 1x */
    conf.os_mode = BMP2_OS_MODE_HIGH_RESOLUTION;

    /* Setting the output data rate */
    conf.odr = BMP2_ODR_125_MS;

    rslt = bmp2_set_config(&conf, &dev);
    bmp2_error_codes_print_result("bmp2_set_config", rslt);

    /* Set normal power mode */
    rslt = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &conf, &dev);
    bmp2_error_codes_print_result("bmp2_set_power_mode", rslt);
	
//	i2c_reg_read(BMP2_I2C_ADDR_PRIM , BMP2_REG_CHIP_ID , (uint8_t *)&rslt, 1);
//	printf("ID : 0x%2X\r\n" , rslt);

}


/*

	PIN#6 : VDDIO : VDD
	PIN#8 : VDD : VDD
	
	PIN#2 (CSB) : HIGH
		CSB to VDDIO : enable I2C
		CSB to GND : enable SPI	
		
	PIN#5 (SDO) : GND	
		7 bit address : 1110 11x
		SDO to GND : address 1110110 (0x76)
		SDO to VDDIO : address 1110111 (0x77)	
	
	PIN#4 : SCL (use external pull up res.)
	PIN#3 : SDA (use external pull up res.)
	
*/

void I2C0_Init(void)	//PC1 : SCL , PC0 : SDA
{
    SYS_ResetModule(I2C0_RST);

    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, MASTER_I2C_SPEED);

//    I2C_SetSlaveAddr(I2C0, 0, LSM6DSL_ADDRESS, 0);   /* Slave Address : 1101011b */

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

	#if defined (ENABLE_I2C_IRQ)
    I2C_EnableInt(MASTER_I2C);
    NVIC_EnableIRQ(MASTER_I2C_IRQn);
	#endif

}

void GPIO_Init (void)
{
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);

    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	
}


void TMR1_IRQHandler(void)
{
//	static uint32_t LOG = 0;
	static uint8_t swap = 0;

	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PB14 ^= 1;
		}

		if ((get_tick() % 250) == 0)
		{
			set_flag(flag_get_sensor_data ,ENABLE);
		}	

		if ((get_tick() % 10000) == 0)
		{
			swap ^= 1;
			set_flag(flag_switch_display ,swap);				
		}	

    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}

void UART02_IRQHandler(void)
{

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_HXTSTB_Msk);

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk|CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk|CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(I2C0_MODULE);

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk)) |
                    (SYS_GPC_MFPL_PC0MFP_I2C0_SDA | SYS_GPC_MFPL_PC1MFP_I2C0_SCL);

   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	UART0_Init();
	GPIO_Init();
	TIMER1_Init();

    I2C0_Init();

	sensor_API_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
		if (is_flag_set(flag_get_sensor_data))
		{
			set_flag(flag_get_sensor_data ,DISABLE);

			sensor_polling();	
		
		}
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
