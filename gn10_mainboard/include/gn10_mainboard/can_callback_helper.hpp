#pragma once
#include <cstdint>

#include "fdcan.h"

/**
 * @brief FDCANのRx FIFO割り込み処理用ヘルパー関数
 *
 * @tparam Bus CANBus / FDCANBus などの update() メソッドを持つクラス
 * @param hfdcan 割り込みが発生した FDCAN ハンドルポインタ
 * @param target_hfdcan 比較対象の FDCAN ハンドルポインタ
 * @param bus 受信時に update() を呼び出す CANバスインスタンス
 * @param fifo_location 対象の FIFO (FDCAN_RX_FIFO0 または FDCAN_RX_FIFO1)
 * @param max_iterations 1回の割り込み処理でパケットを処理する最大回数（無限ループ防止）
 * @return true インスタンスが一致し処理を行った場合, false 一致しなかった場合
 */
template <typename Bus>
inline bool process_fdcan_fifo(
    FDCAN_HandleTypeDef* hfdcan,
    const FDCAN_HandleTypeDef* target_hfdcan,
    Bus& bus,
    uint32_t fifo_location,
    uint8_t max_iterations = 3
)
{
    if (hfdcan->Instance == target_hfdcan->Instance) {
        uint8_t count = 0;
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo_location) > 0 && count < max_iterations) {
            bus.update();
            count++;
        }
        return true;
    }
    return false;
}