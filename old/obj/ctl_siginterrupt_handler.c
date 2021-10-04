/*
 * Handles SIGINTERRUPT
 */
void mpu_ctl_siginterrupt_handler(int signum) {
	printf("\nReceived signal INTERRUPT, aborting..\n");
	close(mpu6050);
	printf("....Done\n");
	exit(signum);
}
