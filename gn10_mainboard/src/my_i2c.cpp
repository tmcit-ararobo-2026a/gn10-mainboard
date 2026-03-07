/**
 * @file my_i2c.cpp
 * @author Gento Aiba
 * @brief 温度センサーと電流センサーの値をI2C通信で取得するクラス
 * @version 0.1
 * @date 2024-04-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "gn10_mainboard/my_i2c.hpp"

/**
 * @brief I2C通信の初期化
 *
 */
void MyI2C::init()
{
    flag_temp    = false;
    flag_current = false;
    HAL_I2C_Master_Transmit(&hi2c1, mcp3421_address << 1, &mcp3421_config, 1, 100);
}

/**
 * @brief 温度センサーの値を取得
 *
 * @param temp 温度センサーの値を格納したい変数のポインタ
 * @return true 新しい値に更新した
 * @return false 新しい値が無い
 */
bool MyI2C::getTemp(uint16_t* temp)
{
    HAL_I2C_Master_Receive(&hi2c1, tmp275_address << 1, tmp275_buff, 2, 100);
    *temp     = (tmp275_buff[0] << 8) | tmp275_buff[1];
    flag_temp = false;
    return true;
}

/**
 * @brief 電流センサーの値を取得
 *
 * @param current 電流センサーの値を格納したい変数のポインタ
 * @return true 新しい値に更新した
 * @return false 新しい値が無い
 */
bool MyI2C::getCurrent(uint16_t* current)
{
    // Decode data
    HAL_I2C_Master_Receive(&hi2c1, (mcp3421_address << 1), mcp3421_buff, 2, 100);
    *current     = (mcp3421_buff[0] << 8) | mcp3421_buff[1];
    flag_current = false;
    return true;
}