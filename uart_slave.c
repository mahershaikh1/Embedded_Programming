/*
  @file uart_slave.c
  @brief Master/slave implementation that communicates over a serial connection. 
  Slave offers remote control of the ZedBoard OLED module. 
  UART parameters: 19200 baud, 8 data, no party, 1 stop bits
 */

#include <liblab2/oled.h>
#include <liblab2/crc.h>
#include <libzed/zed_common.h>
#include <libzed/mmap_regs.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>

static unsigned char stopSlave = 0; /// stop slave loop and exit
static char* uart_regs = NULL; /// UART register mapping

/* @brief Catches CTRL-C, do not call directly
   @param val signal number */
void signal_handler(int val)
{
  // flag stop
  stopSlave = 1;
}

/* @brief Initializes UART
   @return Error codes */
static int uart_init(void)
{
  if (uart_regs)
    {
      // already initialized
      return 1;
    }

  // map
  uart_regs = map_region(UART0_ADDR, 0x1000);
  if (!uart_regs)
    {
      return 1;
    }

  // reset FIFOs and disable interrupts here!
  set_reg_mask(uart_regs, 0x0C, (1 << 0) | (1 << 1));
  clear_reg_mask(uart_regs, 0x0C, (1 << 4));
  return 0;
}

/* @brief Receive a byte from the UART
   @return the received byte */
static unsigned char uart_rxbyte(void)
{
  // Implement your byte receiving code here (blocking),
  // make sure that you wait in this function until a byte is available
  
  // wait for byte
  while (!(read_reg(uart_regs, 0x08) & (1 << 0))) {}

  return read_reg(uart_regs, 0);
}

/* enum to keep track of current state in FSM rx protocol */
typedef enum _FSM_STATE {
    IDLE,
    SOF1,
    SOF2,
    LEN,
    CMD,
    P1,
    P2,
    CRC8,
} FSM_STATE;

/* @brief Execute a command after receiving a packet
   @param command the command byte
   @param params pointer to parameter buffer
   @param paramSize size of parameters */
static int execute_command(unsigned char command, unsigned char* params,
                            unsigned char paramSize)
{
  // Implement your command execution routine here
    int rv = 0;
    // Implement your command execution routine here
    switch(command) {
        case 0x00:
            if (0 != paramSize || NULL != params) {
                printf("Error: command 0 should not have any params - received %d\n", paramSize);
                rv = -1;
            } else {
                //printf("Command 0 received: SCRL_STOP\n");
    lab2_oled_scroll_stop();
            }
            break;
        case 0x01:
            if (0 != paramSize || NULL != params) {
                printf("Error: command 1 should not have any params - received %d\n", paramSize);
                rv = -1;
            } else {
              lab2_oled_step_left();
    }
            break;
        case 0x02:
            if (0 != paramSize || NULL != params) {
                printf("Error: command 2 should not have any params - received %d\n", paramSize);
                rv = -1;
            } else {
                //printf("Command 2 received: STEP_RIGHT\n");
            lab2_oled_step_right(); 
     }
            break;
        case 0x03:
            if (2 != paramSize || NULL == params) {
                printf("Error: command 3 requires 2 params - received %d\n", paramSize);
                rv = -1;
            } else {
                printf("Command 3 received: SCRL_START (%d, %d)\n", *params, *(params+1));
    lab2_oled_scroll_start();
    lab2_oled_set_sspeed((unsigned int)(params[1] | ((unsigned int)params[0]) << 8));
      }
            break;
        case 0x04:
            if (1 != paramSize || NULL == params) {
                printf("Error: command 4 requires 1 param - received %d\n", paramSize);
                rv = -1;
            } else {
                //printf("Command 4 received: SCRN_STATE - %d\n", *params);
              if (params[0] == 0) {
       lab2_oled_screen_off();
    } else {
       lab2_oled_screen_on();
    }
      }
            break;
    }
    return rv;
}

#ifdef DEBUG
#define DBG_PRINT(msg) do { printf("%s", msg); } while(0);
#else
#define DBG_PRINT(msg)
#endif

#define MAX_MSG_LEN 7       // max possible number of bytes in msg protocol 
static FSM_STATE curr_state = IDLE;

// these two are kept to make sure that 
static unsigned int msg_len = 0;
static unsigned int byte_count = 0;

// message needs to be kept for crc check
static unsigned char msg[MAX_MSG_LEN];
/* @brief Resets state machine to IDLE
 *        Also resets all associated variables to default values
 */
static void reset_to_idle(void) 
{
    msg_len = 0;
    byte_count = 0;
    memset(msg, 0, MAX_MSG_LEN * sizeof(unsigned char));
    curr_state = IDLE;
}

/* @brief The slave state machine
   @param newByte new byte received */
static void slave_fsm(unsigned char newByte)
{
  // implement your state machine here, call the execute_command function
  // when needed. Use the crc8() function from liblab2/crc.h to calculate the
  // CRC8 of the message and verify it against what was received    
    
    // first, append new byte to msg array
    msg[byte_count++] = newByte;
   
    // Idle state handler
    if (IDLE == curr_state) {
        DBG_PRINT("IDLE_STATE\n");
        if (newByte == 0x00) {
            curr_state = SOF1;
        } else {
            reset_to_idle();
        }
        return;
    }

    // SOF1 state handler
    if (SOF1 == curr_state) {
        DBG_PRINT("SOF1\n");
        if (0x2A == newByte) {
            curr_state = SOF2;
        } else {
            reset_to_idle();
        }
        return;
    }

    // SOF2 state handler
    if (SOF2 == curr_state) {
        DBG_PRINT("SOF2\n");
        if (msg_len < MAX_MSG_LEN) {
            msg_len = newByte;
            curr_state = LEN;
        } else {
            reset_to_idle();
        }
        return;
    }

    if (LEN == curr_state) {
        DBG_PRINT("LEN\n");
        switch (newByte) {
            case 0:
            case 1:
            case 2:
                curr_state = CRC8;
                break;
            case 3:
                // TODO: separate out params from this shit
            case 4:
                curr_state = P1;
                break;
            default:
                reset_to_idle();
                break;
        }
        return;
    }

    if (P1 == curr_state) {
        DBG_PRINT("P1\n");
        if (0x03 ==  msg[3]) {
            curr_state = P2;
        } else if (0x04 == msg[3]) {
            curr_state = CRC8;
        } else {
            reset_to_idle();
        }
        return;
    }

    if (P2 == curr_state) {
        DBG_PRINT("P2\n");
        curr_state = CRC8;
        return;
    }

    if (CRC8 == curr_state) {
        DBG_PRINT("CRC8\n");
        // implement crc8
        // first, check the length of the bytes to make sure it's all good
        if (byte_count == msg_len) {
            if (newByte == crc8(msg, msg_len-1)) {
                unsigned char param_len; 
                unsigned char *params;
                if (0x03 == msg[3]) {
                    param_len = 2;
                    params = &msg[4];
                } else if (0x04 == msg[3]) {
                    param_len = 1;
                    params = &msg[4];
                } else {
                    param_len = 0;
                    params = NULL;
                }
                DBG_PRINT("Executing command...\n");
                // command should be contained in byte 3, params start at byte 4
                execute_command(msg[3], params, param_len);
            } else {
                printf("CRC Failed: received - 0x%x; crc = 0x%x\n", newByte, crc8(msg, msg_len-1));
            }
        }
        // no matter what, we reset to idle after this state
        reset_to_idle();
        return;
    }

}

int main(void)
{
  int ret;
  unsigned char receivedByte;

  // initialize OLED driver
  ret = lab2_oled_initialize();
  if (ret)
    {
      printf("ERROR: could not initialize OLED\n");
      return 1;
    }

  // initialize UART
  ret = uart_init();
  if (ret)
    {
      printf("ERROR: could not initialize UART\n");
    }

  // setup signal handler
  signal(SIGINT, signal_handler);

  // wait until program is killed with CTRL-C
  while(!stopSlave)
    {
      // receive bytes and process
      receivedByte = uart_rxbyte();
      slave_fsm(receivedByte);
    }

  // de-initialize peripherals and drivers
  lab2_oled_deinitialize();
  unmap_region(uart_regs, 0x1000);

  return 0;
}