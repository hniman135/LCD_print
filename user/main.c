/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"

#include "string.h"
#include "xmc_lcd.h"
#include "touch.h"

/** @addtogroup AT32F403A_periph_template
  * @{
  */

/** @addtogroup 403A_LED_toggle LED_toggle
  * @{
  */

#define DELAY                            100
#define FAST                             1
#define SLOW                             4

uint8_t g_speed = FAST;

void button_exint_init(void);
void button_isr(void);



//===========================================================================





__IO uint16_t adc1_ordinary_value = 0;
crm_clocks_freq_type crm_clocks_freq_struct = {0};
uint16_t point_color;

static void USART1_config(uint32_t baudrate);
static void DMA1_config(void);
static void ADC1_config(void);



//===========================================================================


/**
  * @brief  configure button exint
  * @param  none
  * @retval none
  */
void button_exint_init(void)
{
  exint_init_type exint_init_struct;

  crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
  gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOA, GPIO_PINS_SOURCE0);

  exint_default_para_init(&exint_init_struct);
  exint_init_struct.line_enable = TRUE;
  exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
  exint_init_struct.line_select = EXINT_LINE_0;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  nvic_irq_enable(EXINT0_IRQn, 0, 0);
}

/**
  * @brief  button handler function
  * @param  none
  * @retval none
  */
void button_isr(void)
{
  /* delay 5ms */
  delay_ms(5);

  /* clear interrupt pending bit */
  exint_flag_clear(EXINT_LINE_0);

  /* check input pin state */
  if(SET == gpio_input_data_bit_read(USER_BUTTON_PORT, USER_BUTTON_PIN))
  {
    if(g_speed == SLOW)
      g_speed = FAST;
    else
      g_speed = SLOW;
  }
}

/**
  * @brief  exint0 interrupt handler
  * @param  none
  * @retval none
  */
void EXINT0_IRQHandler(void)
{
  button_isr();
}

//===================================================================================================





#define ADC_VREF                         (3.3)
#define ADC_TEMP_BASE                    (1.26)
#define ADC_TEMP_SLOPE                   (-0.00423)


static void USART1_config(uint32_t baudrate){
	gpio_init_type gpio_init_config;

	//enable USART1 and GPIO clock
	crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

	//config USART1 TX pin
	gpio_default_para_init(&gpio_init_config);
	gpio_init_config.gpio_mode = GPIO_MODE_MUX;
	gpio_init_config.gpio_pins  = GPIO_PINS_9;
	gpio_init(GPIOA, &gpio_init_config);

	//config USART1 parameter
	usart_init(USART1, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
	usart_transmitter_enable(USART1, TRUE);
	usart_enable(USART1, TRUE);

#ifdef Debug
	printf("USART1 config finish\r\n");
#endif

}

static void DMA1_config(void){
	dma_init_type dma1_init_config;
	crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
	dma_reset(DMA1_CHANNEL1);

	dma_default_para_init(&dma1_init_config);

	  dma1_init_config.buffer_size = 1;
	  dma1_init_config.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
	  dma1_init_config.memory_base_addr = (uint32_t)&adc1_ordinary_value;
	  dma1_init_config.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
	  dma1_init_config.memory_inc_enable = FALSE;
	  dma1_init_config.peripheral_base_addr = (uint32_t)&(ADC1->odt);
	  dma1_init_config.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
	  dma1_init_config.peripheral_inc_enable = FALSE;
	  dma1_init_config.priority = DMA_PRIORITY_HIGH;
	  dma1_init_config.loop_mode_enable = TRUE;

	dma_init(DMA1_CHANNEL1, &dma1_init_config);

	dma_channel_enable(DMA1_CHANNEL1, TRUE);

#ifdef Debug
	printf("dma_from_address: %x\r\n", dma1_init_config.peripheral_base_addr);
	printf("dma_to_address: %x\r\n", dma1_init_config.memory_base_addr);
	printf("Temperature address parameter: %x\r\n", &adc1_ordinary_value);
	if(dma1_init_config.memory_base_addr == &adc1_ordinary_value)
	{
		printf("DMA to ADC config success\r\n");
	}
	else
	{
		printf("Error: DMA is not matching addresss with ADC contain parameter\r\n");
	}
	printf("DMA1 config finish\r\n");
#endif

}

//adc1_chan16 connect to internal temperature
static void ADC1_config(void)
{
	adc_base_config_type adc1_base_config;
	crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
	crm_adc_clock_div_set(CRM_ADC_DIV_2);

	adc_reset(ADC1);

	adc_combine_mode_select(ADC_INDEPENDENT_MODE);
	adc_base_default_para_init(&adc1_base_config);
	adc1_base_config.sequence_mode = FALSE;
	adc1_base_config.repeat_mode = TRUE;
	adc1_base_config.data_align = ADC_RIGHT_ALIGNMENT;
	adc1_base_config.ordinary_channel_length = 1;

	adc_base_config(ADC1, &adc1_base_config);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_16, 1, ADC_SAMPLETIME_239_5);
//	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_17, 2, ADC_SAMPLETIME_239_5);
	adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
	adc_dma_mode_enable(ADC1, TRUE);
	adc_tempersensor_vintrv_enable(TRUE);

	adc_enable(ADC1, TRUE);
	adc_calibration_init(ADC1);
	while(adc_calibration_init_status_get(ADC1));
	adc_calibration_start(ADC1);
	while(adc_calibration_status_get(ADC1));

#ifdef Debug
	printf("ADC1_CHAN16 config finish\r\n");
#endif

}

void nvic_configuration(void)
{
  /* 2 bit for pre-emption priority,2 bits for subpriority */
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

//  nvic_irq_enable(LCD_SPI_MASTER_Tx_DMA_IRQn, 0, 2);
}

//----------------------------------------------------------------------
//-- User LCD function



//===================================================================================================

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
	//--System Config
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	system_clock_config();
	at32_board_init();
	at32_led_off(LED2);
	at32_led_off(LED3);
	at32_led_off(LED4);
	USART1_config(115200);
	DMA1_config();
	ADC1_config();
	printf("internal_temperature_sensor \r\n\n");
	adc_ordinary_software_trigger_enable(ADC1, TRUE);

	double tprt_chip = 0;
	char string[] = "Internal Temperature: ";
	//--LCD Config
	lcd_struct = &lcd_dev_struct;
	crm_clocks_freq_get(&crm_clocks_freq_struct);

	lcd_struct->lcd_init();
	lcd_struct->lcd_clear(WHITE);
	delay_sec(3);


	//lcd_struct->lcd_write_string(string, 1, 0,0x0000);
	while(1)
	{
	  at32_led_on(LED2);
	  delay_sec(1);
	  while(dma_flag_get(DMA1_FDT1_FLAG) == RESET);
	  dma_flag_clear(DMA1_FDT1_FLAG);
	  tprt_chip = ((ADC_TEMP_BASE - (double)adc1_ordinary_value * ADC_VREF / 4096) / ADC_TEMP_SLOPE + 25);
	  printf("internal_temperature = %f deg C\r\n", tprt_chip);
	  lcd_struct->lcd_write_num((int)tprt_chip, 1, 26, 0x0000);
	}
}


/**
  * @}
  */

/**
  * @}
  */
