#ifdef __cplusplus
	extern "C" {
#endif
#ifndef _DEMO_SOCKET_H_
#define _DEMO_SOCKET_H_
#include <libmpu6050/mpu6050_core.h>
int mpu_socket_connect(int *sfd, char *host, char *port);
int mpu_socket_sendmsg(int *sfd, char *msg);
int mpu_socket_makemsg(struct mpu_dev *dev, char *msg, char *buf);

#endif /* _DEMO_SOCKET_H_ */
#ifdef __cplusplus
	}
#endif
