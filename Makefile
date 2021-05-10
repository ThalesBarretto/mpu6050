all:
	gcc -DHAVE_INLINE -o mpu_gsl mpu6050.c -O2 -lm -li2c -lgsl
