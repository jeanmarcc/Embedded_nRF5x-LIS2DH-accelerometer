/**  
 *
 * @LISD2H Accelerometer example
 *
 */
/*

Customization:

- sdk_config.h : for TWI, in sdk_config.h, TWI_DEFAULT_CONFIG_CLR_BUS_INIT = 0
- sdk_config.h : for TWI, TWI0_USE_EASY_DMA = 0

*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_drv_twi.h" 
#include "nrf_delay.h"  
#include "ctype.h"  
#include "nrf_drv_gpiote.h"  //accelero 

#define ACC_CS					 	8 
#define ACC_INT2					6  
#define CTS_PIN_NUMBER 		7
#define ACC_INT1					5  
#define ARDUINO_SCL_PIN   10 
#define ARDUINO_SDA_PIN   9 
#define I2C_DELAY     10  // mandatory delay in ms for this interface I2C else the target blocks .... CANNOT be set lower than 10ms

#define SLEEP_MODE_TIMEOUT_SEC  86398  //in seconds, CAN be modified, minimum = 13 seconds and multiples of 13sec -> [13sec, 26sec, 39 sec...] 
#define SLEEP_MODE_TIMEOUT_COUNTER_VALUE  SLEEP_MODE_TIMEOUT_SEC/13  // this define CANNOT be modified

//-- BEGIN ACCELERO PART -------------------------/
#define ACCEL_I2C_ADDR      0x18

#define 	WHO_AM_I	0x0F
#define 	CTRL_REG1	0x20
#define 	CTRL_REG2	0x21
#define 	CTRL_REG3	0x22
#define 	CTRL_REG4	0x23
#define 	CTRL_REG5	0x24
#define 	CTRL_REG6	0x25
#define 	INT1_CFG	0x30
#define 	INT1_SRC	0x31
#define 	INT1_THS	0x32
#define 	INT1_DURATION	0x33
#define 	INT2_CFG	0x34
#define 	INT2_SRC	0x35
#define 	INT2_THS	0x36
#define 	INT2_DURATION	0x37

static uint8_t G_ACCELERO_THRESHOLD_INT1=0x0C;   //threshold for INT1 (wakeup) and INT2 (wakeup) is the same, preferred value = 0x0C;
static uint8_t G_ACCELERO_THRESHOLD_INT2=0x09;   //threshold for INT2 (wakeup) and INT2 (wakeup) is the same, preferred value = 0x09;

//-- END ACCELERO PART -------------------------/

void prepare_sleep_mode (void);
void sleep_mode_enter(void);

static uint16_t sleep_mode_timeout_counter=0; //variable for time to remain active before going Sleep mode, uint16_t unsigned short (16-bit)  

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
#define TWI_INSTANCE_ID     0
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{

    // Initialize timer module.
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function before going in Sleep mode */
void prepare_sleep_mode (void)
{

   	nrf_drv_twi_disable(&m_twi); //disable TWI for Low power mode
		nrf_delay_ms(I2C_DELAY);	
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
							//data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief TWI initialization. =>  I2C
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_LP55231_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_LP55231_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}
/**
 * @brief Function triggered by accelerometer (pin INT1) to wake-up 
 */
void in_pin_handler_INT1(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	
	nrf_drv_twi_enable(&m_twi); //enable TWI

	sleep_mode_timeout_counter = SLEEP_MODE_TIMEOUT_COUNTER_VALUE; //whenever we receive an INT1 then we delay again the time to go to sleep	
	
	//read INT1_SRC to release latch
	uint8_t reg[2];
	reg[0] = INT1_SRC; 
	uint8_t value=0;
	nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, 1, false);
	nrf_delay_ms(I2C_DELAY);	
	while (m_xfer_done == false);
	nrf_drv_twi_rx(&m_twi, ACCEL_I2C_ADDR, &value, 1);
	nrf_delay_ms(I2C_DELAY);
	while (m_xfer_done == false);	
	
}
/**
 * @brief Function triggered by accelerometer (pin INT2) to go in sleep mode
 */
void in_pin_handler_INT2(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

	sleep_mode_timeout_counter--;	
	
	//read INT2_SRC to release latch
	uint8_t reg[2];
	reg[0] = INT2_SRC; 
	uint8_t value=0;
	nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, 1, false);
	nrf_delay_ms(I2C_DELAY);	
	while (m_xfer_done == false);
	nrf_drv_twi_rx(&m_twi, ACCEL_I2C_ADDR, &value, 1);
	nrf_delay_ms(I2C_DELAY);
	while (m_xfer_done == false);	

  if (sleep_mode_timeout_counter<=0)
	{
	  prepare_sleep_mode();
		sd_power_system_off(); //go in Low power mode
	}
	
}

/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    if(!nrf_drv_gpiote_is_init())
        nrf_drv_gpiote_init();
	
		// GPIOTE config for INT1
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = GPIO_PIN_CNF_PULL_Disabled;  //This input pin managed by accelerometer

    err_code = nrf_drv_gpiote_in_init(ACC_INT1, &in_config, in_pin_handler_INT1);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(ACC_INT1, true);
		
		//this is very important for Sleep mode (the GPOITE doesn't wake up itself from Sleep mode)
		nrf_gpio_cfg_sense_input(ACC_INT1, GPIO_PIN_CNF_PULL_Disabled, NRF_GPIO_PIN_SENSE_HIGH); 
		nrf_gpio_cfg_sense_input(ACC_INT2, GPIO_PIN_CNF_PULL_Disabled, NRF_GPIO_PIN_SENSE_HIGH); 
					
		// GPIOTE config for INT2
		err_code = nrf_drv_gpiote_in_init(ACC_INT2, &in_config, in_pin_handler_INT2);
		APP_ERROR_CHECK(err_code);

		nrf_drv_gpiote_in_event_enable(ACC_INT2, true);	
}

/**
 * @brief Function to initialize the accelerometer
 */
void accelero_init()
{
	
		uint8_t reg[2];
		uint8_t err_code;	
	  
  
	// Enable I2C for accelero
		nrf_gpio_cfg_output(ACC_CS);
		nrf_gpio_pin_set(ACC_CS);
		nrf_delay_ms(500); 

//		/***  BEGIN INIT   ***********************************************************/	
    reg[0] = CTRL_REG1;
		reg[1] = 0x00;	
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = CTRL_REG2;
		reg[1] = 0x00;	
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = CTRL_REG3;
		reg[1] = 0x00;	
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = CTRL_REG4;
		reg[1] = 0x00;	
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = CTRL_REG5; 
		reg[1] = 0x00;	
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);
		
		reg[0] = CTRL_REG6;  
		reg[1] = 0x00;		
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);			
		
		reg[0] = INT1_THS;
		reg[1] = 0x00;	
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = INT1_DURATION;
		reg[1] = 0x00;	
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);
		
		reg[0] = INT1_CFG;
		reg[1] = 0x00;			
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);			

		reg[0] = INT2_THS;
		reg[1] = 0x00;	
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = INT2_DURATION;
		reg[1] = 0x00;	
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);
		
		reg[0] = INT2_CFG;
		reg[1] = 0x00;			
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);		
//		/***  END INIT   *************************************************************/
	
		reg[0] = CTRL_REG1;
		reg[1] = 0x2F;		//0x2F Low power mode (10 Hz); all axis //0x5F Low power mode (100 Hz) 
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = CTRL_REG2;
		reg[1] = 0x03;		// High-pass filter for int1 and int2
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = CTRL_REG3;
		reg[1] = 0x40 ;		 // AOI1 interrupt on INT1 
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = CTRL_REG4;
		reg[1] = 0x80  ;		//  Full Scale = +/-2 g 
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = CTRL_REG5; 
		//reg[1] = 0x08 ;		// Latch interrupt on INT1  but NOT on INT2
		//reg[1] = 0x00 ;		// NOT Latch interrupt on INT1  and  NOT on INT2
		reg[1] = 0x0A ;		// Latch interrupt on INT1 / Latch on INT2		
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);
		
		reg[0] = CTRL_REG6; 
		reg[1] = 0x20 ;		// activate INT2
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);			

//******** INT1 Config for WAKE-UP INTERRUPT ********************
		reg[0] = INT1_THS;
		reg[1] = G_ACCELERO_THRESHOLD_INT1;  //1LSb = 16mg @FS=2g  (e.g.-> 0x10*16mg = threshold 256mg)  (e.g.-> 0x40*16mg = threshold 1024mg)  preferred default = 0x10 
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = INT1_DURATION; //beware: on 6 digits only!
		reg[1] = 0x00 ;		 //   [ODR=100HZ -> 1LSB = 10ms]  [ODR=10HZ -> 1LSB = 100ms]
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = INT1_CFG;
		reg[1] = 0xAA  ;		// AND combination of interrupt events  / all interrupts HIGH on XYZ 
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);				

//		reg[0] = WHO_AM_I; 
//		uint8_t value=0;
//		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, 1, false);
//		nrf_delay_ms(I2C_DELAY);	
//		while (m_xfer_done == false);
//		err_code = nrf_drv_twi_rx(&m_twi, ACCEL_I2C_ADDR, &value, 1);
//		nrf_delay_ms(I2C_DELAY);
//		while (m_xfer_done == false);

//******** INT2 Config for GO TO SLEEP INTERRUPT ********************
		reg[0] = INT2_THS;
		reg[1] =  G_ACCELERO_THRESHOLD_INT2;  //1LSb = 16mg @FS=2g  (-> 0x10 * 16mg = 256mg)  (-> 0x40 * 16mg = 898mg)  preferred value = 0x10
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = INT2_DURATION; //beware: on 6 digits only!
		reg[1] = 0x7F ;		 //7F = 12.7 seconds 3F=6.3 seconds   [ODR=10HZ -> 1LSB = 1/10HZ = 100ms]  [ODR=100HZ -> 1LSB = 10ms]  DEFAULT = 0x7F
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);

		reg[0] = INT2_CFG;
		reg[1] = 0x95  ;		// AND combination of interrupt events  / all interrupts LOW XYZ 
		err_code = nrf_drv_twi_tx(&m_twi, ACCEL_I2C_ADDR, reg, sizeof(reg), false);
		nrf_delay_ms(I2C_DELAY);	
		while (m_xfer_done == false);			
//************************************************************

} //end accelero_init()



/**@brief Function for application main entry.
 */
int main(void)
{

    timers_init();
		gpio_init(); //for accelerometer
	
    twi_init(); //Initialization of the I2C
		
		accelero_init();	//accelerometer
		
    // Enter main loop.
    for (;;)
    {
    }
}
