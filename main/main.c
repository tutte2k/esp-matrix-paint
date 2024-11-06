#include <stdio.h>
#include "esp_mac.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "esp_timer.h"
#define DATA_PIN GPIO_NUM_38
#define LATCH_PIN GPIO_NUM_39
#define CLOCK_PIN GPIO_NUM_40
#define JOY_X_PIN ADC1_CHANNEL_4
#define JOY_Y_PIN ADC1_CHANNEL_5
#define JOY_BTN_PIN GPIO_NUM_46
#define LSBFIRST 0
#define MSBFIRST 1
#define MATRIX_SIZE 8
#define ACTION_DELAY 150
static const char *TAG = "LED_Matrix";
struct Position
{
    int x;
    int y;
};
struct Position dotPos = {3, 3};
struct Position dotPos2 = {6, 6};
struct Position litPositions[MATRIX_SIZE * MATRIX_SIZE];
#define MAX_LIT_POSITIONS 64
int litCount = 0;
void setup()
{
    gpio_reset_pin(LATCH_PIN);
    gpio_reset_pin(CLOCK_PIN);
    gpio_reset_pin(DATA_PIN);
    gpio_reset_pin(JOY_BTN_PIN);
    gpio_set_direction(LATCH_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(CLOCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DATA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(JOY_BTN_PIN, GPIO_MODE_INPUT);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(JOY_X_PIN, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(JOY_Y_PIN, ADC_ATTEN_DB_12);
    ESP_LOGI(TAG, "Setup complete");
}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value)
{
    for (int i = 0; i < 8; i++)
    {
        gpio_set_level(dataPin, (bitOrder == LSBFIRST) ? (value & (1 << i)) : (value & (1 << (7 - i))));
        gpio_set_level(clockPin, 1);
        gpio_set_level(clockPin, 0);
    }
}
void updateDotPosition()
{
    int x_value = adc1_get_raw(JOY_X_PIN);
    int y_value = adc1_get_raw(JOY_Y_PIN);
    const int threshold_high = 3000;
    const int threshold_low = 1000;
    if (x_value > threshold_high && dotPos.x < (MATRIX_SIZE - 1))
    {
        dotPos.x++;
        ESP_LOGI(TAG, "Moving right to x=%d", dotPos.x);
    }
    else if (x_value < threshold_low && dotPos.x > 0)
    {
        dotPos.x--;
        ESP_LOGI(TAG, "Moving left to x=%d", dotPos.x);
    }

    if (y_value > threshold_high && dotPos.y > 0)
    {
        dotPos.y--;
        ESP_LOGI(TAG, "Moving up to y=%d", dotPos.y);
    }
    else if (y_value < threshold_low && dotPos.y < (MATRIX_SIZE - 1))
    {
        dotPos.y++;
        ESP_LOGI(TAG, "Moving down to y=%d", dotPos.y);
    }
    dotPos.x = (dotPos.x < 0) ? 0 : (dotPos.x >= MATRIX_SIZE) ? MATRIX_SIZE - 1
                                                              : dotPos.x;
    dotPos.y = (dotPos.y < 0) ? 0 : (dotPos.y >= MATRIX_SIZE) ? MATRIX_SIZE - 1
                                                              : dotPos.y;
}
void delayMicroseconds(uint32_t us)
{
    int64_t start_time = esp_timer_get_time();
    while ((esp_timer_get_time() - start_time) < us)
    {
        //  ( ͡° ͜ʖ ͡°)
    }
}
void displayMatrix()
{
    for (int row = 0; row < MATRIX_SIZE; row++)
    {
        uint8_t rowPattern = (1 << row);
        uint8_t colPattern = 0xFF;
        for (int i = 0; i < litCount; i++)
        {
            if (litPositions[i].y == row)
            {
                colPattern &= ~(1 << litPositions[i].x);
            }
        }
        if (dotPos.y == row)
        {
            colPattern &= ~(1 << dotPos.x);
        }
        gpio_set_level(LATCH_PIN, 0);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, rowPattern);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, colPattern);
        gpio_set_level(LATCH_PIN, 1);
        delayMicroseconds(2000);
    }
    gpio_set_level(LATCH_PIN, 0);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00); // No rows lit
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0xFF); // All columns off
    gpio_set_level(LATCH_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}
void app_main(void)
{
    setup();
    ESP_LOGI(TAG, "Starting LED matrix joystick control");
    TickType_t lastMoveTime = xTaskGetTickCount();
    TickType_t lastClickTime = xTaskGetTickCount();
    while (1)
    {
        TickType_t currentTime = xTaskGetTickCount();
        if ((currentTime - lastMoveTime) >= pdMS_TO_TICKS(ACTION_DELAY))
        {
            updateDotPosition();
            lastMoveTime = currentTime;
        }
        if ((currentTime - lastClickTime) >= pdMS_TO_TICKS(ACTION_DELAY) && gpio_get_level(JOY_BTN_PIN) == 0)
        {
            bool alreadyLit = false;
            for (int i = 0; i < litCount; i++)
            {
                if (litPositions[i].x == dotPos.x && litPositions[i].y == dotPos.y)
                {
                    ESP_LOGI(TAG, "Already lit: x:%d,y:%d", dotPos.x, dotPos.y);
                    alreadyLit = true;
                    litPositions[i] = litPositions[litCount - 1];
                    litCount--;
                    break;
                }
            }

            if (!alreadyLit && litCount < MAX_LIT_POSITIONS)
            {
                ESP_LOGI(TAG, "Now lit: x:%d,y:%d", dotPos.x, dotPos.y);
                litPositions[litCount].x = dotPos.x;
                litPositions[litCount].y = dotPos.y;
                litCount++;
            }
            lastClickTime = currentTime;
        }
        displayMatrix();
    }
}