/* Code to pulse-count for water meter */
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mqtt.h"
#include "ethernet.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "main";

/* Sample rate for the input pin in milliseconds */
#define SENSE_RATE 10

/* Filter depth (all of the previous n bits must match) */
#define SENSE_DEPTH 6

/* Maximum interval between publishing in milliseconds */
#define SENSE_PUB_MAX 30000

/* Minimum interval between publishing in milliseconds */
#define SENSE_PUB_MIN 1000

/* Input pin definitions */
static const gpio_num_t SENSE_PINS[] = {
    GPIO_NUM_2,
    GPIO_NUM_34,
};
#define SENSE_PIN_NUM (sizeof(SENSE_PINS)/sizeof(gpio_num_t))


/* Semaphore to lock counters*/
SemaphoreHandle_t sense_lock = NULL;

/* Public counters */
volatile uint32_t sense_counter[SENSE_PIN_NUM] = {0};


/* Task to sample input pins and perform counting */
static void rt_task(void *pvParameters)
{
    ESP_LOGI(TAG,"Starting rt task");
    /* Last time the task ran */
    TickType_t xLastWakeTime;

    /* Local pin buffers are 32-bit, storing the last 32 samples */
    uint32_t PinBuf[SENSE_PIN_NUM] = {0};
    bool PinState[SENSE_PIN_NUM] = {0};

    /* Local counters are unsigned 32-bit */
    uint32_t PinCount[SENSE_PIN_NUM] = {0}; 

    /* Initialize pin state to current state, configure pin as input */
    for(int i = 0; i < SENSE_PIN_NUM; i++)
    {
        gpio_set_direction(SENSE_PINS[i],GPIO_MODE_INPUT);
        PinState[i] = gpio_get_level(SENSE_PINS[i]);
        PinBuf[i] = (PinState[i]) ? 0xFFFFFFFF : 0;
        ESP_LOGV(TAG,"Initialized sensor %d pad[%d] = %d",i,SENSE_PINS[i],PinState[i]);
    }

    /* Determine the mask required based on the number of bits to consider */
    uint32_t Mask = (1 << SENSE_DEPTH) - 1;
    ESP_LOGV(TAG,"Initialized mask to %08x",Mask);

    /* Initialise the xLastWakeTime variable with the current time. */
    xLastWakeTime = xTaskGetTickCount();

    /* Continuous loop */
    while(1)
    {
        /* Deal with each pin individually */
        for(int i = 0; i < SENSE_PIN_NUM; i++)
        {
            /* Read in the pin */
            bool NewVal = gpio_get_level(SENSE_PINS[i]);

            /* Add it to the bit-array */
            PinBuf[i] = (PinBuf[i] << 1) | NewVal;

            /* Mask by desired depth, compare to zero and mask */
            uint32_t Masked = PinBuf[i] & Mask;

            /* Masked shows all low, pin state was high, falling edge */
            if(0ul == Masked && 1 == PinState[i])
            {
                /* Transition high to low */
                ESP_LOGI(TAG,"Signal %d (pad %d) went from HIGH to LOW",i,SENSE_PINS[i]);
                PinState[i] = 0;
                PinCount[i]++;
            }
            else if(Mask == Masked && 0 == PinState[i])
            {
                /* Transition low to high */
                ESP_LOGI(TAG,"Signal %d (pad %d) went from LOW to HIGH",i,SENSE_PINS[i]);
                PinState[i] = 1;
                PinCount[i]++;
            }
            //ESP_LOGV(TAG,"Signal %d, new val %d, buf %08x, masked %08x",i,NewVal,PinBuf[i],Masked);
        }

        /* Coherently copy out counters */
        xSemaphoreTake(sense_lock,portMAX_DELAY);
        for(int i = 0; i < SENSE_PIN_NUM;i++)
        {
            sense_counter[i] = PinCount[i];
        }
        xSemaphoreGive(sense_lock);

        /* Set task rate to 100hz */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSE_RATE));
    }
}


/* Task to report to MQTT */
static void app_task(void *pvParameters)
{
    ESP_LOGI(TAG,"Starting app task");
    uint32_t PinCount[2][SENSE_PIN_NUM] = {0};
    static char element[128];
    static char msg[1024];
    
    /* Generate topic for this message */
    static char topic[1024];
    sprintf(topic,"raw/%s/counter",mqtt_clid);
    ESP_LOGI(TAG,"Topic is %s",topic);

    /* Store time we last published as now */
    TickType_t LastPub = xTaskGetTickCount();

    /* Initialise the xLastWakeTime variable with the current time. */
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        /* Coherently copy out counters */
        xSemaphoreTake(sense_lock,portMAX_DELAY);
        for(int i = 0; i < SENSE_PIN_NUM;i++)
        {
            /* Store current in [0] */
            PinCount[0][i] = sense_counter[i];
        }
        xSemaphoreGive(sense_lock);

        /* Publish the new numbers if they have changed, or if it has been 30 seconds since we did */
        bool change = 0;
        for(int i = 0; i < SENSE_PIN_NUM; i++)
        {
            /* Compare current [0] to last [1] */
            if(PinCount[0][i] != PinCount[1][i])
            {
                change = 1;
            }
            /* Store last value in [1] */
            PinCount[1][i] = PinCount[0][i];
        }

        TickType_t now = xTaskGetTickCount();
        int32_t delta = pdTICKS_TO_MS(now - LastPub);
        if(delta > SENSE_PUB_MAX)
        {
            change = 1;
        }

        /* Perform a publish */
        if(change)
        {
            /* Build ESP message */
            strcpy(msg,"{");
            for(int i = 0; i < SENSE_PIN_NUM; i++)
            {
                /* Add one more element to the json info */
                sprintf(element," \"Sense%d\": %d",i,PinCount[0][i]);
                strcat(msg,element);

                /* Add a comma if not the last element */
                if(i != (SENSE_PIN_NUM) - 1)
                {
                    strcat(msg,", ");
                }
            }
            /* Close the json */
            strcat(msg," }");
            ESP_LOGI(TAG,"Message to publish: %s",msg);
            ESP_LOGI(TAG,"Publishing, sensors are %d and %d",PinCount[0][0],PinCount[0][1]);

            /* Actually publish the message */
            int id = esp_mqtt_client_publish(mqtt_client, topic, msg, 0, 0, 0);
            ESP_LOGI(TAG,"Published mesage with ID %d",id);

            /* Store the time we last published */
            LastPub = xTaskGetTickCount();
        }

        /* Delay the min time */
        vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(SENSE_PUB_MIN));
    }
}

void app_main() 
{
    /* Start Ethernet */
    eth_init();

    /* Start MQTT */
    mqtt_init();

    ESP_LOGI(TAG,"Starting application section");

    /* Create a semaphore for sample data */
    sense_lock = xSemaphoreCreateMutex();
    if(sense_lock == NULL)
    {
        ESP_LOGE(TAG,"Failed to create semaphore!");
        abort();
    }

    /* Start real-time task */
    xTaskCreatePinnedToCore(rt_task, "rt_task", 2048, NULL, configMAX_PRIORITIES, NULL,1);

    /* Start MQTT transmission task */
    xTaskCreatePinnedToCore(app_task,"app_task",2048,NULL,1,NULL,1);
}