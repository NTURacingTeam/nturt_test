#include "microros_utils.h"

// stm32 include
#include <stm32h7xx_hal.h>

#include "main.h"

// microros include
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include "it_transport.h"
#include "microros_allocators.h"

extern UART_HandleTypeDef huart3;

void error_loop(const rcl_ret_t _rc, const char *_file, const int _line,
                const char *_fun) {
  printf(
      "Microros enter error loop with error code \'%ld\' at file \'%s\', line "
      "\'%d\' in function \'%s\'\n",
      _rc, _file, _line, _fun);

  // enter hard fault
  Error_Handler();
}

void error_once(const rcl_ret_t _rc, const char *_file, const int _line,
                const char *_fun) {
  printf(
      "Microros error once with error code \'%ld\' at file \'%s\', line "
      "\'%d\' in function \'%s\'\n",
      _rc, _file, _line, _fun);

  // blink the led to indicate soft fault
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}

void configure_microros() {
  RCCHECK(rmw_uros_set_custom_transport(
      true, (void *)&huart3, cubemx_transport_open, cubemx_transport_close,
      cubemx_transport_write, cubemx_transport_read));

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  // for some reason true is successful
  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    RCCHECK(RCL_RET_ERROR);
  }
}
