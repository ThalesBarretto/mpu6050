void mpu_log_create(void)
{
/*
struct mpu_archive_header {
	char message[MPU_LOGFILE_MSG_SIZE];
	time_t time_start;
	time_t time_stop;
} logfile_header;

struct mpu_archive_data {
	long long sample_counter;
	int16_t raw[16];
	double data[16][2];

} logfile_data;

char mpu_ctl_log_filename[L_tmpnam] = "";
FILE * mpu_ctl_log_fp;
*/
	mpu_ctl_log_fp = fopen( "mpu6050.log", "w+");
	snprintf(logfile_header.message,MPU_LOGFILE_MSG_SIZE, "%s", "Testing" );
	time(&logfile_header.time_start);
	fwrite(&logfile_header, sizeof(logfile_header), 1, mpu_ctl_log_fp);
	fflush(mpu_ctl_log_fp);
	fclose(mpu_ctl_log_fp);
}
