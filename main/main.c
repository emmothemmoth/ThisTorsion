#include <stdio.h>
#include <math.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_adc/adc_oneshot.h" 


#define SAMPLE_RATE 44100
#define SINE_FREQ_HZ 440.0f 
#define PI 3.14159265359f

#define I2S_BCK_IO 10
#define I2S_WS_IO  11  
#define I2S_DO_IO  13  
#define VOL_POT_CHAN   ADC_CHANNEL_3 
#define DRIVE_POT_CHAN ADC_CHANNEL_4 


_Atomic float global_vol_mult = ATOMIC_VAR_INIT(0.5f);
_Atomic float global_drive_mult = ATOMIC_VAR_INIT(1.0f);
_Atomic bool system_running = true;

void control_knob_task(void *pvParameters) 
{
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12,         
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VOL_POT_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, DRIVE_POT_CHAN, &config));

    printf("Core 0: Knob Task Started.\n");

    while (system_running) 
    {
        int raw_vol = 0, raw_drive = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, VOL_POT_CHAN, &raw_vol));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, DRIVE_POT_CHAN, &raw_drive));
        
        global_vol_mult = (float)raw_vol / 4095.0f; 
        global_drive_mult = 1.0f + (((float)raw_drive / 4095.0f) * 19.0f); 

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    vTaskDelete(NULL);
}

void audio_dsp_task(void *pvParameters) 
{
    i2s_chan_handle_t tx_handle;
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_handle, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, .bclk = I2S_BCK_IO, .ws = I2S_WS_IO,
            .dout = I2S_DO_IO, .din = I2S_GPIO_UNUSED, 
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    size_t buffer_size = 1024; 
    int16_t *audio_buffer = calloc(buffer_size * 2, sizeof(int16_t)); 
    float phase = 0.0f;
    float phase_increment = (2.0f * PI * SINE_FREQ_HZ) / (float)SAMPLE_RATE;
    size_t bytes_written = 0;
    float envelope = 0.0f;

    printf("Core 1: Audio DSP Task Started.\n");

    while (system_running) 
    {
        float current_vol = global_vol_mult;
        float current_drive = global_drive_mult;

        for (int i = 0; i < buffer_size; i++) 
        {
            float sample = sinf(phase);
            
            float abs_sample = fabsf(sample); 
            if (abs_sample > envelope) envelope += 0.01f * (abs_sample - envelope); 
            else envelope += 0.0005f * (abs_sample - envelope); 
            
            float comp_gain = 1.0f;
            if (envelope > 0.5f) {
                float squashed = 0.5f + ((envelope - 0.5f) / 4.0f);
                comp_gain = squashed / envelope; 
            }
            sample = sample * comp_gain * 1.5f;
            sample = sample * current_drive;
            sample = tanhf(sample);
            
            int16_t final_out = (int16_t)(sample * current_vol * 30000.0f);
            audio_buffer[i * 2]     = final_out; 
            audio_buffer[i * 2 + 1] = final_out; 
            
            phase += phase_increment;
            if (phase >= 2.0f * PI) 
            {
                phase -= 2.0f * PI; 
            }
        }
        i2s_channel_write(tx_handle, audio_buffer, buffer_size * 2 * sizeof(int16_t), &bytes_written, portMAX_DELAY);
    }

    ESP_ERROR_CHECK(i2s_channel_disable(tx_handle));
    ESP_ERROR_CHECK(i2s_del_channel(tx_handle));
    free(audio_buffer);
    vTaskDelete(NULL);
}

void app_main(void)
{
    printf("Booting...\n");

    xTaskCreatePinnedToCore(control_knob_task, "Knob_Task", 4096, NULL, 1, NULL, 0);
    
    xTaskCreatePinnedToCore(audio_dsp_task, "Audio_Task", 8192, NULL, 5, NULL, 1);

    vTaskDelay(pdMS_TO_TICKS(30000));

    printf("Shutting down...\n");
    system_running = false; 
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("System Shutdown Complete.\n");
}

//void app_main(void)
//{
//
//}
