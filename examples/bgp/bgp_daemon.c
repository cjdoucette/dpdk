#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <unistd.h>

#define BUF_SIZE	128

int
main(void)
{
	unsigned char buf[BUF_SIZE];
	unsigned char *data;
	ssize_t numbytes;
	int s;

	s = socket(PF_INET, SOCK_RAW, IPPROTO_TCP);
	if (s == -1) {
		perror("socket");
		return -1;
	}

	memset(buf, 0, BUF_SIZE);
	numbytes = recvfrom(s, buf, BUF_SIZE, 0, NULL, NULL);
	if (numbytes < 0) {
		perror("recvfrom");
		close(s);
		return -1;
	}

	data = buf + sizeof(struct iphdr) + sizeof(struct tcphdr);
	printf("%s\n", data);

	close(s);
	return 0;
}
