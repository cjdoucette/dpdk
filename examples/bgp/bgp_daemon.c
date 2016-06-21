#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <unistd.h>

#define BUF_SIZE	1500
#define TCP_BGP_PORT	179

int
main(void)
{
	unsigned char buf[BUF_SIZE];
	struct tcphdr *tcph;
	unsigned char *data;
	ssize_t numbytes;
	int s;

	s = socket(AF_INET, SOCK_RAW, IPPROTO_TCP);
	if (s == -1) {
		perror("socket");
		return -1;
	}

	while (1) {
		memset(buf, 0, BUF_SIZE);
		numbytes = recvfrom(s, buf, BUF_SIZE, 0, NULL, NULL);
		if (numbytes < 0) {
			perror("recvfrom");
			close(s);
			return -1;
		}
		tcph = (struct tcphdr *)(buf + sizeof(struct ip));
		data = buf + sizeof(struct iphdr) + sizeof(struct tcphdr);

		if (ntohs(tcph->dest) != 22)
			printf("%zd bytes received (%hu): %s\n", numbytes,
				ntohs(tcph->dest), data);
	}

	close(s);
	return 0;
}
