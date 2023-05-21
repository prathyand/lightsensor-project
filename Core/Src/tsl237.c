#include <stdio.h>
#include <string.h>

#include "main.h"
#include "tsl237.h"
#include "temperature.h"


#define NUM_SAMPLES 10

extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

uint32_t tsl237_done = 0;

uint32_t buf[NUM_SAMPLES+2] = {0};

uint32_t tsl237_readsensor() {
  char print_buffer[100];
  long long sum = 0;
  //float average_period;
  uint32_t average_period;
  int i;
  // sensor_power(POWER_ON);
  //tsl237_vdd_on();  // Power on the sensor
  HAL_Delay(200); // Wait a second to allow the sensor to stabilize
  //  MX_DMA_Init();
  //  HAL_TIM_Base_Init(&htim2);
  uint32_t start_time = HAL_GetTick();
  tsl237_done = 0;

  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) buf, NUM_SAMPLES+2);

  while (1) {
    if (tsl237_done != 0) {
      for (i=3;i<NUM_SAMPLES+2;i++) {
        if (buf[i] >= buf[i-1]) {
          sum += (long long) (buf[i] - buf[i-1]);
          //          printf("GTE %d\n\r",(int) (buf[i]-buf[i-1]));
        }
        else {
          sum += (long long) ((0xffffffff - buf[i-1]) + buf[i]);
          /* printf("ARR = %u\n\r",(unsigned int) htim2.Instance->ARR); */
          /* printf("buf[i-1] = %u\n\r",(unsigned int) buf[i-1]); */
          /* printf("buf[i] = %u\n\r",(unsigned int) buf[i]); */
          /* printf("LT %d\n\r",(int) (((uint32_t) htim2.Instance->ARR) - buf[i-1] + buf[i])); */
        }
      }
      //    average_period = (float) sum/(NUM_SAMPLES-1);  // Compute the average
      average_period = sum/(NUM_SAMPLES-1);  // Compute the average
      break;
    }
    else if ((HAL_GetTick()-start_time) > MAX_SAMPLE_TIME) {
      //      printf("time = %d\n\r",(int) (HAL_GetTick()-start_time));
      // If the time has exceeded MAX_SAMPLE_TIME, abort the sampling
      HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
      for (i=1;i<NUM_SAMPLES;i++) {
        if (buf[i]) {
          if (buf[i] >= buf[i-1]) {
            sum += (long long) (buf[i] - buf[i-1]);
          }
          else {
            sum += (long long) ((0xffffffff - buf[i-1]) + buf[i]);
          }
          //          printf("%d %d %d %d\n\r", i, (int) buf[i-1], (int) buf[i], (int) (buf[i]-buf[i-1]));
        }
        else {
          sprintf(print_buffer,"Note: Short Read of %d samples after %d seconds",i-1,MAX_SAMPLE_TIME/1000);
//          write_log_data(&fs,print_buffer);
          break;
        }
      }
      average_period = sum/(i-1);  // Compute the average given the shortened number of edges.

      break;
    }

  }

  return(average_period);
}
