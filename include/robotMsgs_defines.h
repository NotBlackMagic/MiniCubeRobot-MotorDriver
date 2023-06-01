#ifndef DRIVER_ROBOTMSGS_DEFINES_H_
#define DRIVER_ROBOTMSGS_DEFINES_H_

/********************************************************/
/*			ROBOT MESSAGES PACKET FORMAT				*/
/********************************************************/
//|   u16    | u32  |   u16    | n*u8 |
//|----------|------|----------|------|
//| Topic ID | nsec | Frame ID | Data |

#define ROBOT_MSG_PKT_HEADER_SIZE				0x08

typedef enum {
	RobotTopic_Battery = 0x00,
	RobotTopic_Odom = 0x01,
	RobotTopic_Range = 0x02,
	RobotTopic_Contact = 0x03,
	RobotTopic_Twist = 0x04,
	RobotTopic_LaserScan = 0x05,
	RobotTopic_Drive = 0x06,
	RobotTopic_Time = 0x07
} RobotTopicID;

#endif /* DRIVER_ROBOTMSGS_DEFINES_H_ */
