#ifndef DRIVER_ROBOTMSGS_DEFINES_H_
#define DRIVER_ROBOTMSGS_DEFINES_H_

/********************************************************/
/*			ROBOT MESSAGES PACKET FORMAT				*/
/********************************************************/
//|  1 byte  |  1 byte  |  1 byte  | n bytes | 2 bytes |
//|----------|----------|----------|---------|---------|
//| Src ID   | Msgs ID  | Pkt Len  | Data    | CRC     |

/********************************************************/
/*					COMMAND MESSAGES					*/
/********************************************************/
#define ROBOT_CMD_REBOOT						0x00
#define ROBOT_CMD_ABORT							0x01
#define ROBOT_CMD_TWIST							0x02
#define ROBOT_CMD_TRANSFORM						0x03

/********************************************************/
/*					INFO MESSAGES						*/
/********************************************************/
#define ROBOT_MSGS_BATTERY						0x00
#define ROBOT_MSGS_ODOM							0x01
#define ROBOT_MSGS_RANGE						0x02
#define ROBOT_MSGS_CONTACT						0x03
#define ROBOT_MSGS_DRIVE						0x04
#define ROBOT_MSGS_TIME							0x05

/********************************************************/
/*					DEBUG MESSAGES						*/
/********************************************************/
#define ROBOT_DBG_DRIVE							0x44

#endif /* DRIVER_ROBOTMSGS_DEFINES_H_ */
