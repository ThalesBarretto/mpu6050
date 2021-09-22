PROGS = mock
OBJS  = dev_mpu_comm	\
	dev_mpu		\
	dev_mpu_conn	\
	dev_mpu_mtx

dev_mpu_mtx.o: dev_mpu_mtx.c
	gcc -c dev_mpu_mtx.c -lm -std=c11 -Wall

dev_mpu_conn.o: dev_mpu_conn.c
	gcc -c dev_mpu_conn.c -li2c -Wall

dev_mpu_comm.o: dev_mpu_comm.c
	gcc -c dev_mpu_comm.c -li2c -Wall

dev_mpu.o: dev_mpu.c dev_mpu_comm.o
	gcc -c dev_mpu.c -li2c -Wall

test: test_mtx.c
	gcc -o test_mtx test_mtx.c dev_mpu_mtx.c -lm

all: dev_mpu.o dev_mpu_comm.o dev_mpu_conn.o dev_mpu_mtx.o
	gcc -o mock mock.c dev_mpu.o dev_mpu_comm.o dev_mpu_conn.o dev_mpu_mtx.o -li2c -Wall -lm

clean:
	rm mock dev_mpu.o 
