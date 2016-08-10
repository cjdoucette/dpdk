#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <libmnl/libmnl.h>
#include <linux/rtnetlink.h>
#include <arpa/inet.h>
#include <time.h>

#define BUF_SIZE		1500
#define TCP_BGP_PORT		179
#define ETH_P_EXPERIMENTAL	0x88B5

/*
 * What kind of messages and flags we want to send depend on what the
 * BGP wants to express in terms of adding, overwriting, replacing,
 * etc. Here are some sample commands and flags from the ip application:
 *
 *	if (matches(*argv, "add") == 0)
 *		return iproute_modify(RTM_NEWROUTE, NLM_F_CREATE|NLM_F_EXCL,
 *				      argc-1, argv+1);
 *	if (matches(*argv, "change") == 0 || strcmp(*argv, "chg") == 0)
 *		return iproute_modify(RTM_NEWROUTE, NLM_F_REPLACE,
 *				      argc-1, argv+1);
 *	if (matches(*argv, "replace") == 0)
 *		return iproute_modify(RTM_NEWROUTE, NLM_F_CREATE|NLM_F_REPLACE,
 *				      argc-1, argv+1);
 *	if (matches(*argv, "prepend") == 0)
 *		return iproute_modify(RTM_NEWROUTE, NLM_F_CREATE,
 *				      argc-1, argv+1);
 *	if (matches(*argv, "append") == 0)
 *		return iproute_modify(RTM_NEWROUTE, NLM_F_CREATE|NLM_F_APPEND,
 *				      argc-1, argv+1);
 *	if (matches(*argv, "test") == 0)
 *		return iproute_modify(RTM_NEWROUTE, NLM_F_EXCL,
 *				      argc-1, argv+1);
 *	if (matches(*argv, "delete") == 0)
 *		return iproute_modify(RTM_DELROUTE, 0,
 *				      argc-1, argv+1);
 *
 * For this sample BGP daemon, we'll copy "add," "change," and "delete."
 */

#define IFNAME	"vEth0_0"
#define DST	"1.2.3.4"
#define GW	"5.6.7.8"
#define PREFIX	32

static void
send_eth_frame(void)
{
	int s;
	size_t len = 0;

	struct ifreq ifr;
	struct ifreq ifr_mac;
	struct sockaddr_ll socket_address;

	struct ether_header *eh;

	char *buf;
        struct nlmsghdr *nlh;
        struct rtmsg *rtm;
        uint32_t prefix = PREFIX, seq;
	in_addr_t dst, gw;
        int family = AF_INET;

	buf = malloc(MNL_SOCKET_BUFFER_SIZE);
	if (!buf) {
		printf("malloc");
		return;
	}

    	s = socket(AF_PACKET, SOCK_RAW, IPPROTO_TCP);
	if (s == -1) {
		perror("socket");
		return;
	}

	eh = (struct ether_header *)buf;
	/* Get the index of the interface to send on. */
	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_name, IFNAME, IFNAMSIZ - 1);
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
		perror("ioctl SIOCGIFINDEX");

	/* Get the MAC address of the interface to send on. */
	memset(&ifr_mac, 0, sizeof(struct ifreq));
	strncpy(ifr_mac.ifr_name, IFNAME, IFNAMSIZ - 1);
	if (ioctl(s, SIOCGIFHWADDR, &ifr_mac) < 0)
		perror("ioctl SIOCGIFHWADDR");

	/* Ethernet header */
	memcpy(eh->ether_shost, ifr_mac.ifr_hwaddr.sa_data, ETH_ALEN);
	memset(eh->ether_dhost, 0xff, ETH_ALEN);

	/* Ethertype field */
	eh->ether_type = htons(ETH_P_EXPERIMENTAL);

	len += sizeof(struct ether_header);

	/* Packet data */
	if (!inet_pton(family, DST, &dst)) {
		perror("inet_pton");
		return;
	}
	if (!inet_pton(family, GW, &gw)) {
		perror("inet_pton");
		return;
	}

	prefix = PREFIX;

	nlh = mnl_nlmsg_put_header(buf + len);
	nlh->nlmsg_type = RTM_NEWROUTE;
	nlh->nlmsg_flags = NLM_F_REQUEST | NLM_F_CREATE | NLM_F_EXCL;
	printf("flags: %04hx\n", nlh->nlmsg_flags);
	nlh->nlmsg_seq = seq = time(NULL);

	rtm = mnl_nlmsg_put_extra_header(nlh, sizeof(struct rtmsg));
	rtm->rtm_family = family;
	rtm->rtm_dst_len = prefix;
	rtm->rtm_src_len = 0;
	rtm->rtm_tos = 0;
	rtm->rtm_protocol = RTPROT_STATIC;
	rtm->rtm_table = RT_TABLE_MAIN;
	rtm->rtm_type = RTN_UNICAST;
	/* If no gateway, this could be RT_SCOPE_LINK. */
	rtm->rtm_scope = RT_SCOPE_UNIVERSE;
	rtm->rtm_flags = 0;

	mnl_attr_put_u32(nlh, RTA_DST, dst);
	mnl_attr_put_u32(nlh, RTA_OIF, 1);
	mnl_attr_put_u32(nlh, RTA_GATEWAY, gw);

	len += nlh->nlmsg_len;

	/* Index of the network device */
	socket_address.sll_ifindex = ifr.ifr_ifindex;
	/* Address length*/
	socket_address.sll_halen = ETH_ALEN;
	/* Destination MAC */
	memset(socket_address.sll_addr, 0xff, ETH_ALEN);

	/* Send packet */
	if (sendto(s, buf, len, 0, (struct sockaddr*)&socket_address,
		   sizeof(struct sockaddr_ll)) < 0)
	    perror("sendto");
}

int
main(void)
{
	unsigned char buf[BUF_SIZE];
	struct tcphdr *tcph;
	unsigned char *data;
	ssize_t numbytes;
	int s;

	send_eth_frame();

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
