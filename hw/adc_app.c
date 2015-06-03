#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/ioctl.h>

#include "adc.h"

// ./adc_app /dev/adc
int main(int argc, const char *argv[])
{
	int fd;
	int ret = 0;
	int vol;
	
	if (argc < 2){
		fprintf(stderr, "Usage: %s <adc device>\n", argv[0]);
		exit(EXIT_FAILURE);
	}
	
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
	
	while (1){
		ret = read(fd, &vol, sizeof(vol));
		if (-1 == ret){
			perror("Fail to open");
			break;
		}
		
		printf("vol = %d\n", vol);
		printf("vol = %.2f\n", vol * 3.3 / 4096);
		sleep(1);
	}
	
	close(fd);
	
	return 0;
}

