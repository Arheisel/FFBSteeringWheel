#pragma once
#include <cstdint>
#include "config.h"

class ButtonReader
{
public:
    void init();
    void update();
    uint16_t get_buttons() const { return debounced_; }

private:
    uint16_t history_[DEBOUNCE_READS] = {};
    uint8_t history_idx_ = 0;
    uint16_t debounced_ = 0;
    int dma_tx_chan_ = -1;
    int dma_rx_chan_ = -1;
    bool dma_active_ = false;
    uint8_t tx_dummy_[2] = {0, 0};
    uint8_t rx_buf_[2] = {0, 0};
};
