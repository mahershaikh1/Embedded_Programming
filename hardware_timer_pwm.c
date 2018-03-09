/*
  @file gpio.c
  @brief Generates 1ms 50% duty cycle PWM signal using the hardware AXI Timer
 */

#include <sys/signal.h>
#include <stdio.h>
#include <libzed/axi_gpio.h>
#include <libzed/zed_common.h> 
#include <libzed/mmap_regs.h>

#define TIMER_REGION_LEN 0x1F
#define TIMER_CTRL_SET_MASK (1 << 9) | (1 << 2) 
#define TIMER_CTRL_CLEAR_MASK (1 << 0) | (1 << 1)

#define COUNTER_MAX 0xFFFFFFFF
/// stop infinite loop flag
static unsigned char stopLoop = 0;

/* @brief Handle signals
   @param num Signal Number */
static void signal_handler(int num)
{
  stopLoop = 1;
}

int main(void)
{


  // handle SIGINT (ctrl-c)
  signal(SIGINT, signal_handler);

  // initialize peripherals here, use libzed, lookup .h files in
  // /usr/share/EECE45343/include/libzed
  struct axi_gpio_data * gpio = axigpio_get(LED_GPIO_ADDR, 0);
  char * timer_regs;
  timer_regs = map_region(TIMER0_ADDR, TIMER_REGION_LEN);  
  if (0 == timer_regs) {
  printf("map_region failed\n");
  return -1;
  }
  
  // initializing timer peripheral
  clear_reg_mask(timer_regs, 0x0, 0xFFFFFFFF);
  clear_reg_mask(timer_regs, 0x10, 0xFFFFFFFF);
  printf("Control B reg: %d\n",read_reg(timer_regs, 0x10));
  set_reg_mask(timer_regs, 0x0, TIMER_CTRL_SET_MASK);
  set_reg_mask(timer_regs, 0x10, TIMER_CTRL_SET_MASK);
  printf("Control B reg: %d\n",read_reg(timer_regs, 0x10));
  
  // setting period, frequency by using load register
  set_reg_mask(timer_regs, 0x4, COUNTER_MAX - 99998);
  set_reg_mask(timer_regs, 0x14, COUNTER_MAX - 49998);

  set_reg_mask(timer_regs, 0x0, (1 << 7));
  set_reg_mask(timer_regs, 0x10, (1 << 7));

  printf("Control A reg: %d\n",read_reg(timer_regs, 0x0));
  printf("Control B reg: %d\n",read_reg(timer_regs, 0x10));
  printf("Load A reg: %u\n",read_reg(timer_regs, 0x04));
  printf("Load B reg: %u\n",read_reg(timer_regs, 0x14));

  if (0 != axigpio_group_output(gpio, 0xFF, 0)) {
  printf("Set gpio output failed\n");
  return -1;
  }

  
  while (!stopLoop)
    {


    }


  return 0;
}