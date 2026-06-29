// =========================================================================
// I2C DMA Driver for AS5600
// =========================================================================
// RP2040 I2C DMA is complex because the I2C controller doesn't have
// a simple block-read DMA interface. We must feed the DATA_CMD register
// with specific command words.
// =========================================================================

#include "i2c_dma.h"
#include "config.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "pico/time.h"

// I2C DATA_CMD register flags
#define I2C_CMD_RESTART 0x400
#define I2C_CMD_STOP 0x200
#define I2C_CMD_READ 0x100

// Resolve I2C peripheral from config-derived instance number
static auto *const I2C_PORT = i2c_get_instance(I2C_INSTANCE);

bool I2CDMA::init_peripheral_()
{
    i2c_init(I2C_PORT, I2C_FREQ_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    // Initialize AS5600 CONF register (16-bit write: PM=00, SF=10, FTH=100, WD=0)
    uint8_t config_data[3] = {AS5600_REG_CONF, AS5600_CONF_VALUE_H, AS5600_CONF_VALUE_L};

    for (int i = 0; i < AS5600_CONF_RETRY_COUNT; i++)
    {
        int wrote = i2c_write_blocking(I2C_PORT, AS5600_I2C_ADDR, config_data, 3, false);
        if (wrote == 3)
        {
            return true;
        }
        sleep_ms(1); // Brief sleep before retry
    }

    return false;
}

bool I2CDMA::init()
{
    if (!init_peripheral_())
        return false;

    dma_tx_chan_ = dma_claim_unused_channel(true);
    dma_rx_chan_ = dma_claim_unused_channel(true);

    // Prepare the command buffer for reading 3 bytes starting from STATUS register (0x0B)
    tx_cmd_buf_[0] = AS5600_REG_STATUS;              // Write register address
    tx_cmd_buf_[1] = I2C_CMD_RESTART | I2C_CMD_READ; // Restart and read byte 1 (Status)
    tx_cmd_buf_[2] = I2C_CMD_READ;                   // Read byte 2 (Angle High)
    tx_cmd_buf_[3] = I2C_CMD_STOP | I2C_CMD_READ;    // Read byte 3 (Angle Low) and Stop

    // Configure TX DMA (writes commands to DATA_CMD)
    tx_cfg_ = dma_channel_get_default_config(dma_tx_chan_);
    channel_config_set_transfer_data_size(&tx_cfg_, DMA_SIZE_16);
    channel_config_set_dreq(&tx_cfg_, i2c_get_dreq(I2C_PORT, true));
    channel_config_set_read_increment(&tx_cfg_, true);
    channel_config_set_write_increment(&tx_cfg_, false);

    // Configure RX DMA (reads data from DATA_CMD)
    // We only want to capture the 3 read bytes. The first command is a write,
    // which doesn't produce RX data.
    rx_cfg_ = dma_channel_get_default_config(dma_rx_chan_);
    channel_config_set_transfer_data_size(&rx_cfg_, DMA_SIZE_8); // We only care about the lower 8 bits
    channel_config_set_dreq(&rx_cfg_, i2c_get_dreq(I2C_PORT, false));
    channel_config_set_read_increment(&rx_cfg_, false);
    channel_config_set_write_increment(&rx_cfg_, true);

    // Enable DMA IRQ for RX completion (once, not per-read)
    dma_channel_set_irq0_enabled(dma_rx_chan_, true);

    return true;
}

void I2CDMA::start_read()
{
    // Set target address
    i2c_get_hw(I2C_PORT)->enable = 0;
    i2c_get_hw(I2C_PORT)->tar = AS5600_I2C_ADDR;
    i2c_get_hw(I2C_PORT)->enable = 1;

    // Clear any pending TX abort condition (prevents bus staying stuck after NACK)
    (void)i2c_get_hw(I2C_PORT)->clr_tx_abrt;

    // RP2040-E13 safe DMA abort: disable IRQ, abort, clear spurious flag, re-enable
    dma_channel_set_irq0_enabled(dma_rx_chan_, false);
    dma_channel_abort(dma_tx_chan_);
    dma_channel_abort(dma_rx_chan_);
    dma_channel_acknowledge_irq0(dma_rx_chan_);
    dma_channel_set_irq0_enabled(dma_rx_chan_, true);

    // Drain any stale data from the I2C RX FIFO
    while (i2c_get_hw(I2C_PORT)->status & I2C_IC_STATUS_RFNE_BITS)
    {
        (void)i2c_get_hw(I2C_PORT)->data_cmd;
    }

    dma_channel_configure(dma_rx_chan_, &rx_cfg_,
                          rx_buf_,                         // dest
                          &i2c_get_hw(I2C_PORT)->data_cmd, // src
                          3,                               // read 3 bytes
                          false);

    dma_channel_configure(dma_tx_chan_, &tx_cfg_,
                          &i2c_get_hw(I2C_PORT)->data_cmd, // dest
                          tx_cmd_buf_,                     // src
                          4,                               // 4 commands total
                          false);

    // Start both
    dma_start_channel_mask((1u << dma_tx_chan_) | (1u << dma_rx_chan_));
}

bool I2CDMA::handle_isr()
{
    if (dma_channel_get_irq0_status(dma_rx_chan_))
    {
        dma_channel_acknowledge_irq0(dma_rx_chan_);
        return true;
    }
    return false;
}

bool I2CDMA::reset_bus()
{
    // 1. Abort any hanging DMA
    dma_channel_abort(dma_tx_chan_);
    dma_channel_abort(dma_rx_chan_);

    // 2. Disable I2C peripheral to take manual control of pins
    i2c_deinit(I2C_PORT);

    // 3. Configure pins as raw GPIO outputs to bit-bang a recovery
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_SIO);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_I2C_SCL, GPIO_OUT);
    gpio_set_dir(PIN_I2C_SDA, GPIO_IN);
    gpio_pull_up(PIN_I2C_SDA); // Need pull up to read it

    // 4. Toggle SCL up to 9 times to force the stuck slave to release SDA
    for (int i = 0; i < 9; i++)
    {
        // If SDA goes high, the bus is free!
        if (gpio_get(PIN_I2C_SDA))
        {
            break;
        }
        gpio_put(PIN_I2C_SCL, 0);
        sleep_us(5);
        gpio_put(PIN_I2C_SCL, 1);
        sleep_us(5);
    }

    // 5. Generate a hardware STOP condition (SDA low to high while SCL is high)
    gpio_set_dir(PIN_I2C_SDA, GPIO_OUT);
    gpio_put(PIN_I2C_SDA, 0);
    sleep_us(5);
    gpio_put(PIN_I2C_SCL, 1);
    sleep_us(5);
    gpio_put(PIN_I2C_SDA, 1);
    sleep_us(5);

    // 6. Re-initialize I2C peripheral and re-apply AS5600 config
    return init_peripheral_();
}
