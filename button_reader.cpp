// =========================================================================
// Button Reader — SPI + DMA, DEBOUNCE_READS-deep debounce
// =========================================================================
// Reads 16 buttons from 2x HCF4021B shift registers via hardware SPI.
// Latch pulse → SPI DMA read 2 bytes → rolling buffer → debounce.
// Pull-ups: 0 = pressed, so we invert the result.
// =========================================================================

#include "button_reader.h"
#include "config.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// Resolve SPI peripheral from config-derived instance number
static auto *const SPI_PORT = SPI_INSTANCE == 0 ? spi0 : spi1;

void ButtonReader::init()
{
    // Initialize SPI peripheral
    spi_init(SPI_PORT, SPI_FREQ_HZ);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_RX, GPIO_FUNC_SPI);

    // Limit SCK drive strength and slew rate to reduce EMI/ringing
    gpio_set_drive_strength(PIN_SPI_SCK, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_slew_rate(PIN_SPI_SCK, GPIO_SLEW_RATE_SLOW);

    // Latch pin — manual GPIO
    gpio_init(PIN_SPI_LATCH);
    gpio_set_dir(PIN_SPI_LATCH, GPIO_OUT);

    // Limit LATCH drive strength and slew rate as well
    gpio_set_drive_strength(PIN_SPI_LATCH, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_slew_rate(PIN_SPI_LATCH, GPIO_SLEW_RATE_SLOW);

    gpio_put(PIN_SPI_LATCH, 0);

    // Claim DMA channels
    dma_tx_chan_ = dma_claim_unused_channel(true);
    dma_rx_chan_ = dma_claim_unused_channel(true);
    dma_channel_config tx_cfg = dma_channel_get_default_config(dma_tx_chan_);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&tx_cfg, spi_get_dreq(SPI_PORT, true));
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);

    dma_channel_config rx_cfg = dma_channel_get_default_config(dma_rx_chan_);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&rx_cfg, spi_get_dreq(SPI_PORT, false));
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, true);

    dma_channel_configure(dma_rx_chan_, &rx_cfg,
                          rx_buf_,                   // dest
                          &spi_get_hw(SPI_PORT)->dr, // src
                          2,                         // count
                          false);                    // don't start yet

    dma_channel_configure(dma_tx_chan_, &tx_cfg,
                          &spi_get_hw(SPI_PORT)->dr, // dest
                          tx_dummy_,                 // src
                          2,                         // count
                          false);                    // don't start yet
}

void ButtonReader::update()
{
    if (dma_active_)
    {
        // Check if finished
        if (dma_channel_is_busy(dma_rx_chan_))
        {
            return; // Still running, do nothing this frame
        }

        dma_active_ = false;

        // 3. Assemble 16-bit value and invert (pull-up: 0 = pressed → 1 = pressed)
        uint16_t raw = static_cast<uint16_t>((rx_buf_[0] << 8) | rx_buf_[1]);
        raw = ~raw; // Invert: 0=pressed becomes 1=pressed

        // 4. Store in rolling buffer
        history_[history_idx_] = raw;
        history_idx_ = (history_idx_ + 1) % DEBOUNCE_READS;

        // 5. Debounce: require DEBOUNCE_READS consecutive identical reads to change state in either direction.
        uint16_t all_pressed = 0xFFFF;
        uint16_t any_pressed = 0;
        for (uint8_t i = 0; i < DEBOUNCE_READS; i++)
        {
            all_pressed &= history_[i];
            any_pressed |= history_[i];
        }
        debounced_ = (debounced_ & any_pressed) | all_pressed;
    }

    if (!dma_active_)
    {
        // 1. Latch: pulse HIGH then LOW to capture parallel button states
        gpio_put(PIN_SPI_LATCH, 1);
        // Brief delay for parallel load (tW).
        // At 3.3V, HCF4021B can need up to ~500ns to settle. 1us is safe.
        busy_wait_us_32(1);
        gpio_put(PIN_SPI_LATCH, 0);

        // Brief delay for setup time (tSU) before the first clock edge.
        busy_wait_us_32(1);

        // 2. Restart DMA channels
        dma_channel_set_read_addr(dma_rx_chan_, &spi_get_hw(SPI_PORT)->dr, false);
        dma_channel_set_write_addr(dma_rx_chan_, rx_buf_, false);
        dma_channel_set_trans_count(dma_rx_chan_, 2, false);

        dma_channel_set_read_addr(dma_tx_chan_, tx_dummy_, false);
        dma_channel_set_write_addr(dma_tx_chan_, &spi_get_hw(SPI_PORT)->dr, false);
        dma_channel_set_trans_count(dma_tx_chan_, 2, false);

        // Start both channels simultaneously
        dma_start_channel_mask((1u << dma_tx_chan_) | (1u << dma_rx_chan_));
        dma_active_ = true;
    }
}
