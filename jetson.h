#ifndef JETSON_H_
#define JETSON_H_

#include "hal.h"
#include "ch.h"
#include "leds.h"
#include "lines.h"
#include "config.h"

/**
 * @defgroup Jetcom Jetson communication
 * @brief
 * All functions, variables, threads and macros used to communicate
 * with Jetson.
 * @details
 * 3 threads communicating with Nvidia Jetson tx2 via serial
 * communication (USART). Sends commands to other parts of program
 * to next processing.
 * @{
 */

#define JETSON_SERIAL &SD6

#define LOAD_JETSON_VALUES 1
#define SAVE_JETSON_VALUES 2


#define JETSON_MOVE_COMMAND 255
#define KICK_COMMAND 254
#define START_STOP_COMMAND 251
#define LINE_CALIBRATION_COMMAND 250
#define INIT_COMMAND 249
#define RESET_COMMAND 248
#define DRIBLER_COMMAND 247

#define START_STOP 3
#define DRIBLER_ON_OFF 4
#define KICK 2

extern uint16_t ball_degree;
extern int16_t robot_speed;
extern int16_t robot_azimuth;

/**
 * @brief
 * Send message to [Jetson write thread]
 * (@ref THD_FUNCTION(JetsonSaveThread, arg)).
 * @param command
 * @returns chMsgSend(jetson_write_thread, command);
 *
 *
 */
msg_t send_jetson(msg_t command);

/**
 * @brief
 * Send message to [Jetson save thread]
 * (@ref THD_FUNCTION(JetsonSaveThread, arg)) which load movement
 * values from jetson to main variables.
 * @returns chMsgSend(jetson_save_thread, LOAD_JETSON_VALUES);
 * @warning
 * movement values load to main variables
 *
 */
msg_t get_jetson_values(void);

/**
 * @brief
 * ## Init of all 3 Jetson processing threads:
 * + [Jetson write thread](@ref THD_FUNCTION(JetsonWriteThread, arg))
 * + [Jetson save thread](@ref THD_FUNCTION(JetsonSaveThread, arg))
 * + [Jetson read thread](@ref THD_FUNCTION(JetsonReadThread, arg))
 *
 */
void jetson_init(void);


/**
 * @}
 */
#endif
