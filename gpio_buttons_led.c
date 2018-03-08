/*
  @file gpio.c
  @brief GPIO Manipulation using buttons and LEDs
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>

// LED GPIO Address
#define LED_GPIO_ADDR 0x41200000

// BUTTIONS Address
#define BTN_GPIO_ADDR 0x41220000


#define GPIO_MAP_LEN 0xFF

// register offsets
#define REG_OFFSET(REG, OFF) ((REG)+(OFF))

// map GPIO peripheral; base address will be mapped to gpioregs pointer
static int map_gpio(unsigned int** gpioregs, unsigned int gpio_addr)
{
  int fd;

  if (!gpioregs || !gpio_addr)
    {
      // invalid pointers
      return 1;
    }

  int map_len = GPIO_MAP_LEN;

  fd = open( "/dev/mem", O_RDWR);
  if (fd <= 0)
  {
	  // fail
	  return 1;
  }

  *gpioregs = mmap(NULL, map_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)gpio_addr);



  // success
  return 0;
}

int main(int argc, char** argv)
{
  unsigned int *gpioregs;
  unsigned int *gpiobtns;
  char str[50];
  int N = 0;
  int i = 0;

  // try to setup LEDs
  if (map_gpio(&gpioregs, LED_GPIO_ADDR))
  {
      // failed to setup
      return 1;
  }

  // try to setup buttons
  if (map_gpio(&gpiobtns, BTN_GPIO_ADDR))
  {
      // failed to setup
      return 1;
  }



  // set LEDs to output
  *REG_OFFSET(gpioregs, 4) = 0x00;

  // set BTNs to input
  *REG_OFFSET(gpiobtns, 4) = (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4);

  // set LEDs off
  *REG_OFFSET(gpioregs, 0) = 0x00;

  while (1)
    {

	  N = *REG_OFFSET(gpiobtns, 0);
	  if (N==1)
	  {
		  printf("CENTER button pushed \n");
		  N = 0;
		  usleep(500000);
	  }
	  if (N == 2)
	  {
		  printf("DOWN button pushed  \n");
		  N = 0;
		 *REG_OFFSET(gpioregs, 0) = (1<<3)|(1<<2);
		 usleep(500000);
		 *REG_OFFSET(gpioregs, 0) = 0x00;

	  }
	  if (N == 4)
	  {
		  printf("LEFT button pushed \n");
		  N = 0;
		  *REG_OFFSET(gpioregs, 0) = (1<<7)|(1<<6);
		  usleep(500000);
		  *REG_OFFSET(gpioregs, 0) = 0x00;
	  }
	  if (N == 8)
	  {
		  printf("RIGHT button pushed  \n");
		  N = 0;
		  *REG_OFFSET(gpioregs, 0) = (1<<1)|(1<<0);
		  usleep(500000);
		  *REG_OFFSET(gpioregs, 0) = 0x00;
	  }
	  if (N == 16)
	  {
		  printf("UP button pushed  \n");
		  N = 0;
		  *REG_OFFSET(gpioregs, 0) = (1<<5)|(1<<4);
		  usleep(500000);
		  *REG_OFFSET(gpioregs, 0) = 0x00;
	  }

    }

  return 0;
}