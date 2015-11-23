#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/pwr.h>

// USB Code

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0xF539,
    .idProduct = 0xF539,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};


static const struct usb_endpoint_descriptor data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x81,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = 0xFF,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 2,

    .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,

    .interface = ifaces,
};

static const char *serial_str = (const char*)0x08004000;

static const char *usb_strings[] = {
    "James Pallister",
    "Medium speed energy monitor",
    (const char *) 0x08004000,
};

void dma_setup();
void adc_setup();
void error_condition();

// Power data ///////////////////////////////////////////////////////

#define INSTANT_AVG_BITS    5
#define INSTANT_AVG_NUM     (1<<INSTANT_AVG_BITS)

typedef struct {
    uint64_t energy_accum;
    uint64_t elapsed_time;
    unsigned peak_power;
    unsigned peak_voltage;
    unsigned peak_current;
    unsigned n_samples;
    uint64_t avg_current;
    uint64_t avg_voltage;
} accumulated_data;

typedef struct {
    unsigned voltage;
    unsigned current;
    unsigned average_voltage;
    unsigned average_current;
    uint64_t current_time;
} instant_data;

/* Sample at 25kS/s (2 periods/sample), 1 out of 3 channels active for now.

   One buffer will be sent every 20 microseconds (3.5 microseconds
   xfer time for 128 bits, 4 microseconds for 144 bits).

   The divide-by-4 is necessary to adjust for the 1/2-sysclk APB1 clock,
   furter divided by 2 due to prescaler divisor being equal to 1.  The resulting
   period (in cycles of 84 MHz APB1 clock) results in a frequency of 40 kHz
   for I or V component in alternation, leading to a complete measurement being
   available every 2100 APB1 cycles, or 50 microseconds.   */
int tperiod = 168000000/4/40000;

typedef struct {
    accumulated_data accum_data;
    instant_data id;


    int idx;
    int running; // Are we collecting measurements
    int number_of_runs;

    // Implement a circular buffer in data_bufs
    int trigger_port, trigger_pin;

    int assigned_adc;

    unsigned short lastI, lastV;
    unsigned lastP;

    unsigned short avgI[INSTANT_AVG_NUM], avgV[INSTANT_AVG_NUM];
    unsigned short avg_ptr;

    unsigned char chans[2];
} measurement_point;

measurement_point m_points[4] = {0};

int adc_to_mpoint[3] = {-1, -1, -1};

/* Variable to hold the last reading of TIM5.  */
volatile unsigned int tim5_value = 0;

// USB communication globals ////////////////////////////////////////
usbd_device *usbd_dev;

uint8_t control_buffer[128] __attribute__((aligned (16)));

unsigned versionNumber=14;

/////////////////////////////////////////////////////////////////////

void exti_setup(int m_point)
{
    int i;

    nvic_disable_irq(NVIC_EXTI0_IRQ);
    nvic_disable_irq(NVIC_EXTI1_IRQ);
    nvic_disable_irq(NVIC_EXTI2_IRQ);
    nvic_disable_irq(NVIC_EXTI3_IRQ);
    nvic_disable_irq(NVIC_EXTI4_IRQ);
    nvic_disable_irq(NVIC_EXTI9_5_IRQ);
    nvic_disable_irq(NVIC_EXTI15_10_IRQ);
    // timer_disable_counter(TIM3);
    exti_reset_request(EXTI0 | EXTI1 | EXTI2 | EXTI3 | EXTI4 | EXTI5 | EXTI6  | EXTI7
            | EXTI8 | EXTI9 | EXTI10 | EXTI11 | EXTI12 | EXTI13 | EXTI14  | EXTI15);

    // if(m_points[m_point].trigger_port == -1)
        // return;

    for(i = 0; i < 4; ++i)
    {
        exti_select_source(m_points[i].trigger_pin, m_points[i].trigger_port);
        exti_set_trigger(m_points[i].trigger_pin, EXTI_TRIGGER_BOTH);
        exti_enable_request(m_points[i].trigger_pin);
        gpio_mode_setup(m_points[i].trigger_port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, m_points[i].trigger_pin);

        switch(m_points[i].trigger_pin)
        {
            case 1<<0: nvic_enable_irq(NVIC_EXTI0_IRQ); break;
            case 1<<1: nvic_enable_irq(NVIC_EXTI1_IRQ); break;
            case 1<<2: nvic_enable_irq(NVIC_EXTI2_IRQ); break;
            case 1<<3: nvic_enable_irq(NVIC_EXTI3_IRQ); break;
            case 1<<4: nvic_enable_irq(NVIC_EXTI4_IRQ); break;
            case 1<<5:
            case 1<<6:
            case 1<<7:
            case 1<<8:
            case 1<<9: nvic_enable_irq(NVIC_EXTI9_5_IRQ); break;
            case 1<<10:
            case 1<<11:
            case 1<<12:
            case 1<<13:
            case 1<<14:
            case 1<<15: nvic_enable_irq(NVIC_EXTI15_10_IRQ); break;
        }
    }
}

/* Forward declaration of buffer cleanup function.  */
void buffer_cleanup ();

/* Start a measurement.  */
void start_measurement(int m_point)
{
    systick_interrupt_disable ();

    /* Raise GPIO D14 to indicate start of measurement.  */
    gpio_set (GPIOD, GPIO14);

    /* Enable SPI1 peripheral.  */
    spi_enable(SPI1);

    /* FIXME/TODO: FORNOW: Enable ADC IRQs before power-up to catch spurious ones.
       Ideally, we should clear any pending requests before enabling the IRQs.  */
    nvic_enable_irq(NVIC_ADC_IRQ);

    m_points[m_point].running = 1;

    m_points[m_point].accum_data.energy_accum = 0;
    m_points[m_point].accum_data.elapsed_time = 0;
    m_points[m_point].accum_data.peak_power = 0;
    m_points[m_point].accum_data.peak_voltage = 0;
    m_points[m_point].accum_data.peak_current = 0;
    m_points[m_point].accum_data.n_samples = 0;
    m_points[m_point].accum_data.avg_voltage = 0;
    m_points[m_point].accum_data.avg_current = 0;
    m_points[m_point].lastP = 0;

    switch(m_points[m_point].assigned_adc)
    {
        case 0:
            adc_set_regular_sequence(ADC1, 1, m_points[m_point].chans);
            adc_power_on(ADC1);
            break;
        case 1:
            adc_set_regular_sequence(ADC2, 1, m_points[m_point].chans);
            adc_power_on(ADC2);
            break;
        case 2:
            adc_set_regular_sequence(ADC3, 1, m_points[m_point].chans);
            adc_power_on(ADC3);
            break;
        case -1:
            error_condition(); return;
        default:
            error_condition(); return;
    }

    /* Clean up the transmission buffers.  */
    buffer_cleanup (m_points[m_point].assigned_adc);
}

void stop_measurement(int m_point)
{
    m_points[m_point].running = 0;
    m_points[m_point].number_of_runs++;

    gpio_clear (GPIOD, GPIO14);

    switch(m_points[m_point].assigned_adc)
    {
        case 0: adc_off(ADC1); break;
        case 1: adc_off(ADC2); break;
        case 2: adc_off(ADC3); break;
        case -1:
            error_condition(); return;
        default:
            error_condition(); return;
    }

    systick_interrupt_enable ();
}

void flash_serial(char b1, char b2, char b3, char b4)
{
    uint32_t base_addr = (uint32_t) serial_str;

    flash_unlock();
    flash_erase_sector(1, FLASH_CR_PROGRAM_X32);

    flash_program_byte(base_addr+0, b1);
    flash_program_byte(base_addr+1, b2);
    flash_program_byte(base_addr+2, b3);
    flash_program_byte(base_addr+3, b4);

    flash_program_byte(base_addr+4, 0x0);
    flash_lock();
}

static int usbdev_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
        uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    int i;

    (void)complete;
    (void)buf;
    (void)usbd_dev;

    tim5_value = timer_get_counter (TIM5);

    switch (req->bRequest) {
    case 0:    // toggle LEDS
        gpio_toggle(GPIOD, GPIO13);
        *len = 0;
        break;
    case 1:     // Start
    {
        gpio_set(GPIOD, GPIO12);

        start_measurement(req->wValue-1);
        *len = 0;
        break;
    }
    case 2:     // Stop
    {
        gpio_clear(GPIOD, GPIO12);
        stop_measurement(req->wValue-1);
        *len = 0;
        break;
    }
    case 3:     // Set serial
    {
        if(*len != 0)
            return 0;

        flash_serial(req->wValue & 0xFF, req->wValue >> 8, req->wIndex & 0xFF, req->wIndex >> 8);

        break;
    }
    case 4:     // Set Trigger
    {
        if(*len != 0)
            return 0;

        int m_point = (req->wValue >> 8) - 1;

        gpio_toggle(GPIOD, GPIO14);

        switch(req->wValue & 0xFF)
        {
            case 'A': m_points[m_point].trigger_port = GPIOA; break;
            case 'B': m_points[m_point].trigger_port = GPIOB; break;
            case 'C': m_points[m_point].trigger_port = GPIOC; break;
            case 'D': m_points[m_point].trigger_port = GPIOD; break;
            case 'E': m_points[m_point].trigger_port = GPIOE; break;
            case 'F': m_points[m_point].trigger_port = GPIOF; break;
            case 'G': m_points[m_point].trigger_port = GPIOG; break;
            case 'H': m_points[m_point].trigger_port = GPIOH; break;
            default:
                m_points[m_point].trigger_port = -1; break;
        }

        m_points[m_point].trigger_pin = 1 << (req->wIndex & 0xFF);

        if(m_points[m_point].trigger_port != GPIOA)
            gpio_toggle(GPIOD, GPIO12);

        exti_setup(m_point);
        break;
    }
    case 6:     // Get energy
    {
        *len = sizeof(accumulated_data);
        *buf = (uint8_t*)&m_points[req->wValue-1].accum_data;
        break;
    }
    case 7:     // Map ADC to measurement point
    {
        int adc = req->wIndex;
        int m_point = req->wValue - 1;

        m_points[m_point].assigned_adc = adc;
        adc_to_mpoint[adc] = m_point;
        break;
    }
    case 8:     // Is running
    {
        *len = sizeof(m_points[req->wValue-1].running);
        *buf = (uint8_t*)&m_points[req->wValue-1].running;
        break;
    }
    case 9:     // Get number of runs
    {
        *len = sizeof(m_points[req->wValue-1].number_of_runs);
        *buf = (uint8_t*)&m_points[req->wValue-1].number_of_runs;
        break;
    }
    case 10:    // Clear number of runs
    {
        m_points[req->wValue-1].number_of_runs = 0;
        break;
    }
    case 11:    // Get instantaneous
    {
        int m_point = req->wValue - 1;
        int tot_current = 0, tot_voltage = 0, i;
        measurement_point *mp = &m_points[m_point];

        mp->id.current = mp->lastI;
        mp->id.voltage = mp->lastV;

        for(i = 0; i < INSTANT_AVG_NUM; ++i)
        {
            tot_current += mp->avgI[i];
            tot_voltage += mp->avgV[i];
        }

        mp->id.average_current = tot_current >> INSTANT_AVG_BITS;
        mp->id.average_voltage = tot_voltage >> INSTANT_AVG_BITS;

        mp->id.current_time = mp->accum_data.elapsed_time;

        *len = sizeof(instant_data);
        *buf = (uint8_t*)&m_points[req->wValue-1].id;
        break;
    }
    case 12:    // Get version
    {
        *len = sizeof(versionNumber);
        *buf = (uint8_t*)&versionNumber;
        break;
    }
    case 13:    // Get serial
    {
        *len = 4;
        *buf = (uint8_t*)serial_str;
        break;
    }

    default:
        return 0;
    }
    return 1;
}

static void usb_reset_cb()
{
    int i;

    for(i = 0; i < 4; ++i)
    {
        m_points[i].running = 0;
        m_points[i].trigger_port = -1;
        m_points[i].trigger_pin = -1;
        m_points[i].assigned_adc = -1;
    }
}

static void usbdev_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    // usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, usbdev_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_INTERRUPT, 64, NULL);

    usbd_register_control_callback(
                usbd_dev,
                USB_REQ_TYPE_VENDOR | 3,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                usbdev_control_request);

    usbd_register_reset_callback(usbd_dev, usb_reset_cb);
}

/* Set up TIM2 to drive the ADC firing.
   ZC FIXME/TODO: switch to a smaller timer to free a 32-bit timer
   (cf. RM0090 section 18: TIM3/TIM4 are 16-bit.)  */
void timer2_setup ()
{
    rcc_peripheral_enable_clock (&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);

    timer_disable_counter (TIM2);
    timer_reset (TIM2);
    timer_set_mode (TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period (TIM2, tperiod - 1);
    timer_set_prescaler (TIM2, 0);
    timer_set_clock_division (TIM2, TIM_CR1_CKD_CK_INT);		/* FIXME/TODO: Redundant wrt. 'timer_set_mode'.  */
    timer_set_master_mode (TIM2, TIM_CR2_MMS_UPDATE);
    timer_enable_preload (TIM2);
    timer_enable_counter (TIM2);
}

/* Set up TIM5 to provide a microsecond clock with fairly monotonic advance.
   Rollover will happen after 2^32 microseconds == 4294.967 s == 1h 11m 34.80s
   while measurement runs are in the few seconds range. */
void timer5_setup (void)
{
    rcc_peripheral_enable_clock (&RCC_APB1ENR, RCC_APB1ENR_TIM5EN);

    timer_disable_counter (TIM5);
    timer_reset (TIM5);
    timer_set_mode (TIM5, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period (TIM5, 1000000000 - 1);
    /* Set prescaler to 83 (non-zero value means TIM5 will be driven by full
       APB1 frequency which is half the CPU clock.)  */
    timer_set_prescaler (TIM5, (168 / 2) - 1);
    timer_set_master_mode (TIM5, TIM_CR2_MMS_UPDATE);
    /* Generate update events only on overflow.  */
    timer_update_on_overflow (TIM5);
    timer_set_repetition_counter (TIM5, 0);
    timer_disable_preload (TIM5);
    timer_enable_counter (TIM5);
}

/* Set up the SPI.  Pin assignment: use plain alternate function 5:
    - SPI1_NSS: PA4
    - SPI1_SCK: PA5
    - SPI1_MISO: PA6
    - SPI1_MOSI: PA7.  */
void spi_setup ()
{
    //gpio_toggle(GPIOD, GPIO15);
    /* Set up GPIOA ports 4, 5, 6 and 7:
        - speed 50 MHz
	- pullup-pulldown (as opposed to open-drain)
	- no pull-up/pull-down direction
	- alternate function #5 (SPI1/SPI2/I2S2/I2S2ext)
	*/
    gpio_mode_setup (GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4 | GPIO5 | GPIO7);
    gpio_mode_setup (GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
    /* Do not set output options on pin PA6 - it's an input.  */
    gpio_set_output_options (GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO4 | GPIO5 | GPIO7);
    gpio_set_af (GPIOA, GPIO_AF5, GPIO4 | GPIO5 | GPIO6 | GPIO7);

    /* Enable SPI1 clock.  */
    rcc_peripheral_enable_clock (&RCC_APB2ENR,
				 /* SPI 1 */
				 RCC_APB2ENR_SPI1EN);

    /* Disable SPI1 peripheral.  */
    spi_disable (SPI1);

    /* Reset SPI1 peripheral.  */
    rcc_peripheral_reset(&RCC_APB2ENR, RCC_APB2RSTR_SPI1RST);

    /* Explicitly disable I2S in favour of SPI operation */
    SPI1_I2SCFGR = 0;

    /* SPI1: disable CRC.  */
    spi_disable_crc(SPI1);

    /* Init master on SPI1:
	- baudrate 1/2 the maximum (84/2 == 42 Mbaud)
	- pull clock high when idle
	- make data stable on falling edge of clock (transition #1)
	- send 8-bit frames
	- send data in MSBit-first order.
	*/
    spi_init_master (SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_2,
		     /* Clock polarity: pull clock high for the peripheral device.  */
		     SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
		     /* CPHA: Clock phase: read on rising edge of clock.  */
		     SPI_CR1_CPHA_CLK_TRANSITION_1,
		     /* DFF: Data frame format (8 or 16 bit): 16 bit.  */
		     SPI_CR1_DFF_16BIT,
		     /* Most or Least Sig Bit First: MSB first. */
		     SPI_CR1_MSBFIRST);

    //spi_enable_software_slave_management(SPI1);
    spi_disable_software_slave_management (SPI1);
    /* Pull /SS high (slave not selected).  */
    /* spi_set_nss_high(SPI1);  */
    spi_enable_ss_output (SPI1);

    /* Clean up status information.  */
    /* spi_clear_mode_fault(SPI1);  */

    //gpio_toggle(GPIOD, GPIO15);
}

/* Use 16-bit DMA xfers.  */
#define USE_16BIT_TRANSFERS

/* Enable DMA interrupts. The priority of the DMA interrupts must be higher
   than the priority of the ADC interrupts to ensure the integrity of DMA
   backlog handling (dma_transceive called from DMA context must run at higher
   priority than the instance called from ADC context.)  */
static void dma_int_enable(void)
{
    /* SPI1 RX on DMA2 Channel 2 */
    nvic_set_priority(NVIC_DMA2_STREAM2_IRQ, 0x10);
    nvic_enable_irq(NVIC_DMA2_STREAM2_IRQ);
    /* SPI1 TX on DMA2 Channel 3 */
    nvic_set_priority(NVIC_DMA2_STREAM3_IRQ, 0x10);
    nvic_enable_irq(NVIC_DMA2_STREAM3_IRQ);
}

/* Set up DMA2.  */
void dma_setup(void)
{
    /* Enable DMA clock.  */
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);
    rcc_periph_clock_enable (RCC_DMA2);
    rcc_periph_clock_enable (RCC_DMA2);

    /* RM0090 10.3.17(1): If the stream is enabled, disable it.
       Disable streams prior to stream programming.  */
    dma_disable_stream (DMA2, DMA_STREAM2);
    dma_disable_stream (DMA2, DMA_STREAM3);

    /* Reset DMA streams.  */
    dma_stream_reset(DMA2, DMA_STREAM2);
    dma_stream_reset(DMA2, DMA_STREAM3);

    /* Rx DMA configuration.  Note it has higher priority to avoid overrun.  */
    /* 10.3.17(2): Set the peripheral port register address.  */
    dma_set_peripheral_address(DMA2, DMA_STREAM2, (uint32_t)&SPI1_DR);

    dma_channel_select (DMA2, DMA_STREAM2, DMA_SxCR_CHSEL_3);
    /* 10.3.17(6): If the peripheral is intended to be the flow controller... */
    /* 10.3.17(7): Configure the stream priority.  */
    dma_set_priority(DMA2, DMA_STREAM2, DMA_SxCR_PL_VERY_HIGH);
    /* 10.3.17(8): Configure the FIFO usage.  */
    /* 10.3.17(9): Configure the data transfer direction...  */
    dma_set_transfer_mode (DMA2, DMA_STREAM2, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    /* 10.3.17(9): ...peripheral and memory incremented/fixed mode,
		  single or burst transactions, peripheral
		  and memory data widths, Circular mode,
		  Double buffer mode and interrupts after
		  half and/or full transfer, and/or errors.*/
#ifdef USE_16BIT_TRANSFERS
    dma_set_peripheral_size(DMA2, DMA_STREAM2, DMA_SxCR_PSIZE_16BIT);
    dma_set_memory_size(DMA2, DMA_STREAM2, DMA_SxCR_MSIZE_16BIT);
#else
    dma_set_peripheral_size(DMA2, DMA_STREAM2, DMA_SxCR_PSIZE_8BIT);
    dma_set_memory_size(DMA2, DMA_STREAM2, DMA_SxCR_MSIZE_8BIT);
#endif
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM2);

    /* Tx DMA configuration.  */
    /* 10.3.17(2): Set the peripheral port register address.  */
    dma_set_peripheral_address(DMA2, DMA_STREAM3, (uint32_t)&SPI1_DR);
    dma_channel_select (DMA2, DMA_STREAM3, DMA_SxCR_CHSEL_3);
    /* 10.3.17(6): If the peripheral is intended to be the flow controller... */
    /* 10.3.17(7): Configure the stream priority.  */
    dma_set_priority(DMA2, DMA_STREAM3, DMA_SxCR_PL_HIGH);
    /* 10.3.17(8): Configure the FIFO usage.  */
    /* 10.3.17(9): Configure the data transfer direction...  */
    dma_set_transfer_mode (DMA2, DMA_STREAM3, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    /* 10.3.17(9): ...peripheral and memory incremented/fixed mode,
		  single or burst transactions, peripheral
		  and memory data widths, Circular mode,
		  Double buffer mode and interrupts after
		  half and/or full transfer, and/or errors.*/
#ifdef USE_16BIT_TRANSFERS
    dma_set_peripheral_size(DMA2, DMA_STREAM3, DMA_SxCR_PSIZE_16BIT);
    dma_set_memory_size(DMA2, DMA_STREAM3, DMA_SxCR_MSIZE_16BIT);
#else
    dma_set_peripheral_size(DMA2, DMA_STREAM3, DMA_SxCR_PSIZE_8BIT);
    dma_set_memory_size(DMA2, DMA_STREAM3, DMA_SxCR_MSIZE_8BIT);
#endif
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM3);

    /* Enable DMA interrupts in general, but disable DMA2 xfer complete ones.  */
    dma_disable_transfer_complete_interrupt(DMA2, DMA_STREAM2);
    dma_disable_transfer_complete_interrupt(DMA2, DMA_STREAM3);

    dma_int_enable ();
}

/* Asssume "previous" transfers both completed.
   Current value is the number of ongoing transfers.  */
unsigned int transceive_status = 0;

/* Backlog entry type.  Assume 16-bit data.  */
typedef struct {
  const unsigned short *tx_base;
  int tx_size;
  const unsigned short *rx_base;
  int rx_size;
} backlog_entry_t;

#define BACKLOG_DEPTH 4
/* Backlog circular buffer.  */
backlog_entry_t backlog[BACKLOG_DEPTH];
int next_to_fill = 0;
int next_to_send = 0;

/* Number of backlog entries.  */
int backlog_used = 0;

int16_t dummy_rx_buf;

#ifdef USE_16BIT_TRANSFERS
static int spi_dma_transceive(const uint16_t *tx_addr, int tx_count, uint16_t *rx_addr, int rx_count)
#else
static int spi_dma_transceive(const uint8_t *tx_addr, int tx_count, uint8_t *rx_addr, int rx_count)
#endif
{
	const unsigned short *tx_buf;
	const unsigned short *rx_buf;
	int tx_len;
	int rx_len;

	gpio_toggle (GPIOB, GPIO15);
	/* Check for 0 length in both tx and rx: if so, process backlog.  */
	if ((tx_count == 0) && (rx_count == 0)) {
		/* Can only do this when there's no active transfer and a backlog  */
		if (transceive_status > 0 || backlog_used == 0)
			error_condition ();

		/* Extract addresses and sizes from backlog.  */
		tx_buf = backlog[next_to_send].tx_base;
		tx_len = backlog[next_to_send].tx_size;
		rx_buf = backlog[next_to_send].rx_base;
		rx_len = backlog[next_to_send].rx_size;

		/* Advance the pointer to the next slot.  Check properties.  */
		next_to_send = (next_to_send + 1) % BACKLOG_DEPTH;
		backlog_used--;
		if ((backlog_used < 0)
		    || (next_to_send == next_to_fill && backlog_used != 0)
		    || (next_to_send != next_to_fill && backlog_used == 0))
			error_condition ();
	}
	/* If a transfer is ongoing, place addresses and sizes in backlog.  */
	else if (transceive_status > 0)
	{
		/* Increment backlog count.  Fail if limit exceeded.  */
		backlog_used++;
		if (backlog_used >= BACKLOG_DEPTH)
			error_condition ();

		/* Copy current request to backlog.  */
		backlog[next_to_fill].tx_base = tx_addr;
		backlog[next_to_fill].tx_size = tx_count;
		backlog[next_to_fill].rx_base = rx_addr;
		backlog[next_to_fill].rx_size = rx_count;

		/* Advance pointer to empty slot.  */
		next_to_fill = (next_to_fill + 1) % BACKLOG_DEPTH;
		return -2;
	}
	else
	{
		/* Normal operation (no backlog, no transfer in progress).  */
		tx_buf = tx_addr;
		tx_len = tx_count;
		rx_buf = rx_addr;
		rx_len = rx_count;
	}

	/* Reset status flag appropriately (both 0 case caught above) */
	transceive_status = 0;
	if (rx_len > 0) {
		transceive_status++ ;
	}
	if (tx_len > 0) {
		transceive_status++;
	}

	/* RM0090 10.3.17(1): If the stream is enabled, disable it.
	   Disable streams prior to stream programming.  */
	dma_disable_stream (DMA2, DMA_STREAM2);
	dma_disable_stream (DMA2, DMA_STREAM3);

	/* Set up rx dma */
	if (rx_len > 0) {
		/* 10.3.17(3): Set the memory address.  */
		dma_set_memory_address(DMA2, DMA_STREAM2, (uint32_t)rx_buf);
		/* 10.3.17(4): Configure the total number of data items to be transferred.  */
		dma_set_number_of_data(DMA2, DMA_STREAM2, rx_len);
		/* 10.3.17(5): Select the DMA channel (request).  */
	}

	/* Set up tx dma */
	if (tx_len > 0) {
		/* 10.3.17(3): Set the memory address.  */
		dma_set_memory_address(DMA2, DMA_STREAM3, (uint32_t)tx_buf);
		/* 10.3.17(4): Configure the total number of data items to be transferred.  */
		dma_set_number_of_data(DMA2, DMA_STREAM3, tx_len);
		/* 10.3.17(5): Select the DMA channel (request).  */
	}

	/* Enable dma transfer complete interrupts.  10.3.17(10): Activate the stream(s). */
	if (rx_len > 0) {
		dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM2);
		dma_enable_stream (DMA2, DMA_STREAM2);
	}
	if (tx_len > 0) {
		dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM3);
		dma_enable_stream (DMA2, DMA_STREAM3);
	}

	/* Start Rx if requested.  */
	if (rx_len > 0) {
		/* gpio_set (GPIOD, GPIO14); */
		spi_enable_rx_dma(SPI1);
	}

	/* Start Tx if requested.  */
	if (tx_len > 0) {
		gpio_set (GPIOD, GPIO15);
		spi_enable_tx_dma(SPI1);
	}

	gpio_clear (GPIOB, GPIO15);
	return 0;
}

/* SPI receive completed with DMA */
void dma2_stream2_isr(void)
{
	// gpio_set(GPIOA,GPIO4);
	if ((DMA2_LISR & DMA_LISR_TCIF2) != 0) {
		DMA2_LIFCR |= DMA_LIFCR_CTCIF2;
	}

	dma_disable_transfer_complete_interrupt(DMA2, DMA_STREAM2);

	spi_disable_rx_dma(SPI1);

	dma_disable_stream(DMA2, DMA_STREAM2);

	/* Decrement the status to indicate one of the transfers is complete */
	transceive_status--;

	/* Mark end of reception.  */
	/* gpio_clear (GPIOD, GPIO14); */

	/* If no transfers are under way, re-enable the ADC interrupts.  */
	if (transceive_status == 0)
	  {
	    /* Check for backlog.  If present, trigger the next transfer.  */
	    if (backlog_used > 0)
		spi_dma_transceive (0, 0, 0, 0);
	    /* Enable the ADC IRQ only after the DMA setup is done.  */
	    nvic_enable_irq (NVIC_ADC_IRQ);
	  }

	/* gpio_clear(GPIOA,GPIO4);  */
}

/* SPI transmit completed with DMA */
void dma2_stream3_isr(void)
{
	//gpio_set(GPIOB,GPIO1);
	if ((DMA2_LISR & DMA_LISR_TCIF3) != 0) {
		DMA2_LIFCR |= DMA_LIFCR_CTCIF3;
	}

	dma_disable_transfer_complete_interrupt(DMA2, DMA_STREAM3);

	spi_disable_tx_dma (SPI1);

	dma_disable_stream (DMA2, DMA_STREAM3);

	//spi_set_nss_high (SPI1);

	/* Decrement the status to indicate one of the transfers is complete */
	transceive_status--;

	/* Mark end of transmission.  */
	gpio_clear (GPIOD, GPIO15);

	/* If no transfers are under way, re-enable the ADC interrupts.  */
	if (transceive_status == 0)
	  {
	    /* Check for backlog.  If present, trigger the next transfer.  */
	    if (backlog_used > 0)
		spi_dma_transceive (0, 0, 0, 0);
	    /* Enable the ADC IRQ only after the DMA setup is done.  */
	    nvic_enable_irq (NVIC_ADC_IRQ);
	  }

	//gpio_clear(GPIOB,GPIO1);
}

void adc_setup()
{
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1 | GPIO2 | GPIO3);
    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2 | GPIO4 | GPIO5);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC2EN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC3EN);
    rcc_peripheral_reset(&RCC_APB2ENR, RCC_APB2RSTR_ADCRST);
    adc_off(ADC1);
    adc_off(ADC2);
    adc_off(ADC3);

    ADC_CCR = 0;
    ADC1_CR1 = 0;
    ADC2_CR1 = 0;
    ADC3_CR1 = 0;
    ADC1_CR2 = 0;
    ADC2_CR2 = 0;
    ADC3_CR2 = 0;

    adc_set_single_conversion_mode(ADC1);
    adc_set_single_conversion_mode(ADC2);
    adc_set_single_conversion_mode(ADC3);

    // Input 1
    uint8_t channels1[] = {2, 12};   // CH2 Voltage, PA2, ADC123
    // uint8_t channels1[] = {ADC_CHANNEL2, ADC_CHANNEL12};   // Voltage, PA2, ADC123
    // uint8_t channels2[] = {ADC_CHANNEL12};  // Ch12 Current, PC2, ADC123
    // Input 2
    // uint8_t channels1[] = {ADC_CHANNEL3};   // Voltage, PA3, ADC123
    // uint8_t channels2[] = {ADC_CHANNEL1};   // Current, PA1, ADC123
    // Input 3
    // uint8_t channels1[] = {ADC_CHANNEL9};   // Voltage, PB1, ADC12
    // uint8_t channels2[] = {ADC_CHANNEL15};  // Current, PC5, ADC12
    // Input self
    // uint8_t channels1[] = {ADC_CHANNEL8};   // Voltage, PB0, ADC12
    // uint8_t channels2[] = {ADC_CHANNEL14};  // Current, PC4, ADC12
    adc_set_regular_sequence(ADC1, 1, channels1);
    adc_set_regular_sequence(ADC2, 1, channels1);
    adc_set_regular_sequence(ADC3, 1, channels1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_15CYC);
    adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_15CYC);
    adc_set_sample_time_on_all_channels(ADC3, ADC_SMPR_SMP_15CYC);
    // Trigger the ADC on the reload (?) of TIM2.
    adc_enable_external_trigger_regular(ADC1,ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);
    adc_enable_external_trigger_regular(ADC2,ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);
    adc_enable_external_trigger_regular(ADC3,ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);

    adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
    adc_set_resolution(ADC2, ADC_CR1_RES_12BIT);
    adc_set_resolution(ADC3, ADC_CR1_RES_12BIT);

    adc_set_right_aligned(ADC1);
    adc_set_right_aligned(ADC2);
    adc_set_right_aligned(ADC3);

    adc_enable_overrun_interrupt(ADC1);
    adc_enable_overrun_interrupt(ADC2);
    adc_enable_overrun_interrupt(ADC3);

    adc_enable_eoc_interrupt(ADC1);
    adc_enable_eoc_interrupt(ADC2);
    adc_enable_eoc_interrupt(ADC3);
    adc_eoc_after_each(ADC1);
    adc_eoc_after_each(ADC2);
    adc_eoc_after_each(ADC3);

    nvic_set_priority(NVIC_ADC_IRQ, 0x20);

    /* Do not enable ADC IRQs yet - wait until measurement starts.  */
}

void exti_timer_setup()
{
    // Timer is used for deboucing (is it?)
    // If output on trigger is the same 1ms later, accept as input
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);
    timer_reset(TIM3);
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(TIM3, 50);	/* Count 50 periods.  */
    timer_set_prescaler(TIM3, 1679);	/* Downscale to PCLK_AHB1 / 1680, or 20 microseconds period.  */
    timer_set_clock_division(TIM3, TIM_CR1_CKD_CK_INT);
    timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE);
    timer_enable_irq(TIM3, TIM_DIER_UIE);

    nvic_set_priority(NVIC_TIM3_IRQ, 0x30);
    nvic_enable_irq(NVIC_TIM3_IRQ);

    nvic_set_priority(NVIC_EXTI0_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI1_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI2_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI3_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI4_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI15_10_IRQ, 0x40);
}

// This should probably reset the board and tell the user
void error_condition()
{
    gpio_set(GPIOD, GPIO13);
    while(1);
}

int main(void)
{
    int c_started=0, n, cpy, i, offset=0;
    short s;

    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
    rcc_peripheral_enable_clock(&RCC_AHB2ENR, RCC_AHB2ENR_OTGFSEN);

    // First want to check our serial. If not set, set it
    // Probably want to check that these are alphanumeric as well
    if(!isalnum(serial_str[0]) || !isalnum(serial_str[1]) || !isalnum(serial_str[2]) || !isalnum(serial_str[3]))
    {
        flash_serial('E', 'E', '0', '0');
    }

    // 1ms tick
    systick_set_reload(168000);
    systick_set_clocksource(STK_CSR_CLKSOURCE);
    systick_counter_enable();
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0xFF);
    systick_interrupt_enable();

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO12);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15 | GPIO14 | GPIO13 | GPIO12);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11 | GPIO13 | GPIO15);
    gpio_clear (GPIOB, GPIO11 | GPIO13 | GPIO15);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);

    for(i = 0;i < 4; ++i)
    {
        m_points[i].assigned_adc = -1;
        m_points[i].trigger_port = -1;
        m_points[i].trigger_pin = -1;
    }

    m_points[0].chans[0] = 2;
    m_points[0].chans[1] = 12;

    m_points[1].chans[0] = 3;
    m_points[1].chans[1] = 1;

    m_points[2].chans[0] = 9;
    m_points[2].chans[1] = 15;

    m_points[3].chans[0] = 8;
    m_points[3].chans[1] = 14;

    dma_setup();
    adc_setup();
    spi_setup ();
    timer2_setup();
    timer5_setup ();
    exti_timer_setup();

    gpio_toggle(GPIOA, GPIO12);


    usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 3, control_buffer, 128);
    usbd_register_set_config_callback(usbd_dev, usbdev_set_config);

    while (1)
    {
        usbd_poll(usbd_dev);
    }
}


int status = -1;

void exti0_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti1_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti2_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti3_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti4_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti9_5_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti15_10_isr () __attribute__ ((weak, alias ("exti_isr")));

void exti_isr()
{
    int i;

    for(i = 0; i < 4; ++i)
        exti_reset_request(m_points[i].trigger_pin);

    if(status == -1 || 1)
    {
        for(i = 0; i < 4; ++i)
        {
            if(m_points[i].trigger_port == -1 || m_points[i].trigger_pin == -1)
                continue;

            if(gpio_get(m_points[i].trigger_port, m_points[i].trigger_pin))
            {
                if(!m_points[i].running)
                    start_measurement(i);
            }
            else
            {
                if(m_points[i].running)
                    stop_measurement(i);
            }
        }

        // Timeout to ignore other spurious edges
        // status = gpio_get(trigger_port, trigger_pin);
        // timer_enable_counter(TIM3);
        // timer_set_counter(TIM3, 0);
    }
}

void tim3_isr()
{
    TIM_SR(TIM3) &= ~TIM_SR_UIF;
    timer_disable_counter(TIM3);
    status = -1;
}

void exit(int a)
{
    while(1);
}

/* ZC: Tx buffer for testing. 128 bits @ 42 Mbit/s ==> at most 328125 xfers/s, 100 kS/s is OK. */
unsigned short tx_buffer[2][9] = {
  { 0x1234, 0x0, 0x0,  0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
  { 0xabcd, 0xffff, 0xffff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }};
unsigned int whichone = 0;

/* Structure type holding a single incremental record.  */
typedef struct {
  unsigned int current : 12;	/* Raw current measurement.  */
  unsigned int voltage : 12;	/* Raw voltage measurement.  */
  unsigned int timeskew : 8;	/* Time delay wrt. main timestamp.  */
} channel_value_t;

/* Structure type holding a trace frame: extensive version: 128 bits  */
typedef struct {
  unsigned int timestamp;	/* Timestamp in microseconds since TIM5 config/overflow.  */
  channel_value_t channel[3];	/* Channel information including time skew.  */
} frame_t;

/* Clean up buffers.  */
void buffer_cleanup (int channel)
{
#if 1
    memset (&tx_buffer[0][3 + 2 * channel], 0, 2 * sizeof (short unsigned int));
    memset (&tx_buffer[1][3 + 2 * channel], 0, 2 * sizeof (short unsigned int));
#endif
}

/* ADC interrupt service routine.  */
void adc_isr()
{
    int m_point;
    measurement_point *mp;
    /* ADC list is fixed.  */
    const unsigned int adcs[3] = {ADC1, ADC2, ADC3};
    int i;
    volatile unsigned int tim5_now;
    register unsigned short tim5_high, tim5_low;

    /* Disable ADC IRQs until DMA+SPI transfer is done.  */
    nvic_disable_irq (NVIC_ADC_IRQ);

    /* Mark start of ISR.  */
    gpio_set (GPIOB, GPIO11);

    /* Get value of TIM5 counter.  */
    tim5_now = timer_get_counter (TIM5);

    /* If a transfer is ongoing, raise an error condition.  */
    if (gpio_get (GPIOD, GPIO15) != 0)
      error_condition ();

    /* Store the TIM5 reading at the start of processing.  */
    tim5_value = tim5_now;
    tim5_low = (unsigned short) (tim5_now & 0xffff);
    tim5_high = (unsigned short) ((tim5_now >> 16) & 0xffff);
    tx_buffer[whichone][1] = tim5_high;
    tx_buffer[whichone][2] = tim5_low;

    for(i = 0; i < 3; ++i)
    {
        if(adc_get_overrun_flag(adcs[i]))
            error_condition();

        if(adc_eoc(adcs[i]))
        {
            unsigned short val;

            m_point = adc_to_mpoint[i];
            if(m_point == -1)
                error_condition();

            // Get measurement point & buffer
            mp = &m_points[m_point];

            // Read ADC
            val = ADC_DR(adcs[i]);

            // Save last value
            if(mp->idx&1)
            {
                mp->lastI = val;
                mp->avgI[mp->avg_ptr] = val;
                mp->avg_ptr = (mp->avg_ptr + 1) & (INSTANT_AVG_NUM - 1);
            }
            else
            {
                mp->lastV = val;
                mp->avgV[mp->avg_ptr] = val;
            }

            // Once we have read both current and voltage
            if((mp->idx & 1) == 1)
            {
                accumulated_data *a_data = &mp->accum_data;
                unsigned short c = mp->lastI;
                unsigned short v = mp->lastV;
                unsigned p = c*v;

		/* Mark start of handling.  */
		gpio_set (GPIOB, GPIO13);

		// Copy current and voltage to DMA buffer.  Add the channel ID+1
		// in the high 4 bits.
		// Store values in currently selected buffer.
		tx_buffer[whichone][3 + (i * 2)] = (c & 0xfff) | (unsigned short) ((i + 1) << 12);
		tx_buffer[whichone][4 + (i * 2)] = (v & 0xfff) | (unsigned short) ((i + 1) << 12);

                if(a_data->elapsed_time > 0) // ignore first sample (lastP = 0)
                {
                    a_data->energy_accum += (p + mp->lastP) / 2; // Trapezoidal integration;
                }

                mp->lastP = p;

                a_data->n_samples += 1;
                a_data->elapsed_time += tperiod;
                a_data->avg_voltage += v;
                a_data->avg_current += c;

                if(p > a_data->peak_power)
                    a_data->peak_power = p;
                if(v > a_data->peak_voltage)
                    a_data->peak_voltage = v;
                if(c > a_data->peak_current)
                    a_data->peak_current = c;

		/* Mark end of handling.  */
		gpio_clear (GPIOB, GPIO13);
            }

            mp->idx = 1-mp->idx;

            // HACK. Here we initialise the next channel to read from
            // because very occasionally the ADC seems to skip the next channel
            // suspect an odd timing bug, but only happens 1/10000000 times.
            unsigned char chan[1];

            // Select odd or even channel
            chan[0] = mp->chans[(mp->idx&1)];
            adc_set_regular_sequence(adcs[i], 1, chan);

            adc_enable_eoc_interrupt(adcs[i]);
        }
    }

    /* Send out the frame and change buffers only if there's something to communicate...  */
    if(1)
    {
	/* Send the currently filled buffer (size is 1/2 of the entire variable.)  */
	spi_dma_transceive (tx_buffer[whichone], sizeof (tx_buffer) / sizeof (unsigned short) / 2,  &dummy_rx_buf, 0);

	/* Flip the buffer index.  */
	whichone = 1 - whichone;
    }

    /* Mark end of ISR.  */
    gpio_clear (GPIOB, GPIO11);
}

int milliseconds = 0;

void sys_tick_handler()
{
    int state = 0;
    int flash=0;

    milliseconds++;

    // Compute state
    if(m_points[0].running || m_points[1].running || m_points[2].running || m_points[3].running)
    {
        state = 1;
        if(m_points[0].running)
            flash |= GPIO12;
        if(m_points[1].running)
            flash |= GPIO13;
	/* GPIO D14 and D15 are used as comm markers.  */
        //if(m_points[2].running)
        //  flash |= GPIO14;
        //if(m_points[3].running)
        //    flash |= GPIO15;
    }

    if(state == 0)
    {
#if 0
        if(milliseconds % 400 < 100)
	  {
	    gpio_clear(GPIOD, GPIO13 | GPIO14 | GPIO15);
            gpio_set(GPIOD, GPIO12);
	  }
        else if(milliseconds % 400 < 200)
	  {
	    gpio_clear(GPIOD, GPIO12 | GPIO14 | GPIO15);
            gpio_set(GPIOD, GPIO13);
	  }
        else if(milliseconds % 400 < 300)
	  {
	    gpio_clear(GPIOD, GPIO12 | GPIO13 | GPIO15);
            gpio_set(GPIOD, GPIO14);
	  }
        else
	  {
	    gpio_clear(GPIOD, GPIO12 | GPIO13 | GPIO14);
            gpio_set(GPIOD, GPIO15);
	  }
#endif
    }
    else if(state == 1)
    {
        if(milliseconds % 200 < 100)
            /* gpio_set(GPIOD, flash) */  ;
        else
	    /* GPIO D14 and D15 are used as comm markers.
	       GPIO D13 is error_condition marker.  */
            gpio_clear(GPIOD, GPIO12 /* | GPIO13 | GPIO14 | GPIO15 */);
    }
}
