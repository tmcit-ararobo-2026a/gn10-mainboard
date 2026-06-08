#include "gn10_mainboard/vesc_can.hpp"

void VescCAN::init()
{
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan3,
        FDCAN_ACCEPT_IN_RX_FIFO0,  // 標準ID：一致しなくても受け取る　あとからかえる
        FDCAN_ACCEPT_IN_RX_FIFO0,  // 拡張ID：一致しなくても受け取る
        FDCAN_FILTER_REMOTE,       // リモート標準：フィルタを通す　あとからかえる
        FDCAN_FILTER_REMOTE        // リモート拡張：フィルタを通す
    );
    rxfilter.IdType       = FDCAN_EXTENDED_ID;
    rxfilter.FilterType   = FDCAN_FILTER_RANGE_NO_EIDM;  // 要相談
    rxfilter.FilterIndex  = 0;
    rxfilter.FilterID1    = 0x00000000;
    rxfilter.FilterID2    = 0x1FFFFFFF;
    rxfilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    // フィルタ設定
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &rxfilter) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        Error_Handler();
    }
    // 割り込み有効
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
}

void VescCAN::send_data(uint32_t can_id, uint8_t* data, uint8_t len)
{
    txheader.BitRateSwitch       = FDCAN_BRS_OFF;
    txheader.DataLength          = len;
    txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txheader.FDFormat            = FDCAN_CLASSIC_CAN;
    txheader.Identifier          = can_id;
    txheader.IdType              = FDCAN_EXTENDED_ID;
    txheader.MessageMarker       = 0;
    txheader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txheader.TxFrameType         = FDCAN_DATA_FRAME;
    // wait until TxFIFO free(TxFIFO is 送信待ち行列)
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) == 0);
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &txheader, data) != HAL_OK) {
        Error_Handler();
    }
}

void VescCAN::receive_data(uint32_t can_id, uint8_t* data, uint8_t len)
{
    uint32_t current_id_comp = 25 | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
    if (can_id == current_id_comp) {
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    }
}

void VescCAN::buffer_append_int16(uint8_t* buffer, int16_t number, int32_t* index)
{
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void VescCAN::buffer_append_int32(uint8_t* buffer, int32_t number, int32_t* index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void VescCAN::buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t* index)
{
    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void VescCAN::buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t* index)
{
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

// set
// controller_id is can_id of VESC
void VescCAN::comm_can_set_duty(uint8_t controller_id, float duty)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);

    uint32_t can_id_duty = controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8);

    send_data(can_id_duty, buffer, send_index);
}

void VescCAN::comm_can_set_current(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);

    uint32_t can_id_current = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
    send_data(can_id_current, buffer, send_index);
}

void VescCAN::comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay)
{
    int32_t send_index = 0;
    uint8_t buffer[6];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    buffer_append_float16(buffer, off_delay, 1e3, &send_index);
    send_data(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void VescCAN::comm_can_set_current_brake(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    send_data(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void VescCAN::comm_can_set_rpm(uint8_t controller_id, float rpm)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    send_data(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void VescCAN::comm_can_set_pos(uint8_t controller_id, float pos)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
    send_data(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

void VescCAN::comm_can_set_current_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    send_data(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
}

/**
 * Same as above, but also sets the off delay. Note that this command uses 6 bytes now. The off
 * delay is useful to set to keep the current controller running for a while even after setting
 * currents below the minimum current.
 */
void VescCAN::comm_can_set_current_rel_off_delay(
    uint8_t controller_id, float current_rel, float off_delay
)
{
    int32_t send_index = 0;
    uint8_t buffer[6];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    buffer_append_float16(buffer, off_delay, 1e3, &send_index);
    send_data(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
}

void VescCAN::comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    send_data(
        controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index
    );
}

void VescCAN::comm_can_set_handbrake(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current, 1e3, &send_index);
    send_data(
        controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index
    );
}

void VescCAN::comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    send_data(
        controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index
    );
}
// 0x1FFFFFFF