#pragma once
#include <cstdint>
#include "hardware/dma.h"

class I2CDMA
{
public:
    bool init();

    // Start a non-blocking DMA transfer to read from the AS5600
    void start_read();

    // Must be called from the DMA completion ISR
    // Returns true if a full read was completed successfully
    bool handle_isr();

    // Aggressively attempt to clear a stuck I2C bus and reset the peripheral
    bool reset_bus();

    // Get the latest read data
    // data[0] = Status Register
    // data[1] = Raw Angle High
    // data[2] = Raw Angle Low
    const uint8_t *get_data() const { return rx_buf_; }

private:
    // Shared I2C peripheral + AS5600 setup (used by init and reset_bus)
    bool init_peripheral_();

    int dma_tx_chan_ = -1;
    int dma_rx_chan_ = -1;

    dma_channel_config tx_cfg_;
    dma_channel_config rx_cfg_;

    // Buffer to hold the I2C DATA_CMD register writes
    // 0: Write register address (0x0B)
    // 1: Restart & Read byte 1
    // 2: Read byte 2
    // 3: Read byte 3 & Stop
    uint16_t tx_cmd_buf_[4];

    // Buffer for received data
    uint8_t rx_buf_[3];
};
