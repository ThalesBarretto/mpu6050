all:
	gcc -DHAVE_INLINE -o mpu_gsl mpu6050.c -O2 -lm -li2c -lgsl

.PHONY test: test.c
	gcc -std=c11 -c matrix.c
	gcc -std=c11 -c dev_mpu_mtx.c
	gcc -std=c11 -c dev_mpu_mtx_pri.c
	gcc -o test test.c dev_mpu_mtx.o 
