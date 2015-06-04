/*
 * ʵ�ֲ���:
 * 1. ׼�������ź�
 *    1.1 ʵ���źŴ�����
 *    1.2 ���źŴ��������ź�
 * 2. ���Լ�����Ϊ�ļ�������
 * 3. ָ�����������첽�źŶ���
 * (���ļ���־λ�����FASYNC��־)
*/
          
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>

#include <signal.h>

#include "adc.h"

int fd;
	
// 1.1 ʵ���źŴ�����
void signal_handler(int signo)
{
	int ret = 0;
	int vol;
	
	printf("%s\n", __func__);
	
	ret = read(fd, &vol, sizeof(vol));
	if (-1 == ret){
		perror("Fail to open");
	} else {
		printf("vol = %.2f\n", vol * 3.3 / 4096);
	}
	
	sleep(1);
}

// ./adc_app /dev/adc
int main(int argc, const char *argv[])
{
	int ret = 0;
	int vol;
	int Oflags;
	
	if (argc < 2){
		fprintf(stderr, "Usage: %s <adc device>\n", argv[0]);
		exit(EXIT_FAILURE);
	}
	
	// 1.2 ���źŴ��������ź�
	signal(SIGIO, signal_handler);
	
	fd = open(argv[1], O_RDWR);
	if (-1 == fd){
		perror("Fail to open");
		exit(EXIT_FAILURE);
	}
	
	ret = ioctl(fd, IOCTL_SET_RESOLUTION, 12);
	if (-1 == ret){
		perror("Fail to open");
		exit(EXIT_FAILURE);
	}
	
	// 2. ���Լ�����Ϊ�ļ�������
	fcntl(fd, F_SETOWN, getpid());
	
	// 3. ָ�����������첽�źŶ���
	Oflags = fcntl(fd, F_GETFL); 
	Oflags |= FASYNC;
	fcntl(fd, F_SETFL, Oflags);
	
	// ��һ�Σ������µ����ݲɼ�
	read(fd, &vol, sizeof(vol));
	
	while (1){
		sleep(1);
	}
	
	close(fd);
	
	return 0;
}

