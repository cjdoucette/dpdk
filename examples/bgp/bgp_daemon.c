#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>

#define BUF_SIZE	1500
#define TCP_BGP_PORT	179
#define ETH_P_EXPERIMENTAL	0x88B5

#if 0
static int iproute_modify(int cmd, unsigned int flags, int argc, char **argv)
{
	struct {
		struct nlmsghdr	n;
		struct rtmsg		r;
		char			buf[1024];
	} req;
	char  mxbuf[256];
	struct rtattr *mxrta = (void *)mxbuf;
	unsigned int mxlock = 0;
	char  *d = NULL;
	int gw_ok = 0;
	int dst_ok = 0;
	int nhs_ok = 0;
	int scope_ok = 0;
	int table_ok = 0;
	int raw = 0;
	int type_ok = 0;
	static int hz;

	memset(&req, 0, sizeof(req));

	req.n.nlmsg_len = NLMSG_LENGTH(sizeof(struct rtmsg));
	req.n.nlmsg_flags = NLM_F_REQUEST|flags;
	req.n.nlmsg_type = cmd;
	req.r.rtm_family = preferred_family;
	req.r.rtm_table = RT_TABLE_MAIN;
	req.r.rtm_scope = RT_SCOPE_NOWHERE;

	if (cmd != RTM_DELROUTE) {
		req.r.rtm_protocol = RTPROT_BOOT;
		req.r.rtm_scope = RT_SCOPE_UNIVERSE;
		req.r.rtm_type = RTN_UNICAST;
	}

	mxrta->rta_type = RTA_METRICS;
	mxrta->rta_len = RTA_LENGTH(0);

	while (argc > 0) {
		if (strcmp(*argv, "src") == 0) {
			inet_prefix addr;

			NEXT_ARG();
			get_addr(&addr, *argv, req.r.rtm_family);
			if (req.r.rtm_family == AF_UNSPEC)
				req.r.rtm_family = addr.family;
			addattr_l(&req.n, sizeof(req), RTA_PREFSRC, &addr.data, addr.bytelen);
		} else if (strcmp(*argv, "as") == 0) {
			inet_prefix addr;

			NEXT_ARG();
			if (strcmp(*argv, "to") == 0) {
				NEXT_ARG();
			}
			get_addr(&addr, *argv, req.r.rtm_family);
			if (req.r.rtm_family == AF_UNSPEC)
				req.r.rtm_family = addr.family;
			addattr_l(&req.n, sizeof(req), RTA_NEWDST, &addr.data, addr.bytelen);
		} else if (strcmp(*argv, "via") == 0) {
			inet_prefix addr;
			int family;

			gw_ok = 1;
			NEXT_ARG();
			family = read_family(*argv);
			if (family == AF_UNSPEC)
				family = req.r.rtm_family;
			else
				NEXT_ARG();
			get_addr(&addr, *argv, family);
			if (req.r.rtm_family == AF_UNSPEC)
				req.r.rtm_family = addr.family;
			if (addr.family == req.r.rtm_family)
				addattr_l(&req.n, sizeof(req), RTA_GATEWAY,
						&addr.data, addr.bytelen);
			else
				addattr_l(&req.n, sizeof(req), RTA_VIA,
						&addr.family, addr.bytelen+2);
		} else if (strcmp(*argv, "from") == 0) {
			inet_prefix addr;

			NEXT_ARG();
			get_prefix(&addr, *argv, req.r.rtm_family);
			if (req.r.rtm_family == AF_UNSPEC)
				req.r.rtm_family = addr.family;
			if (addr.bytelen)
				addattr_l(&req.n, sizeof(req), RTA_SRC, &addr.data, addr.bytelen);
			req.r.rtm_src_len = addr.bitlen;
		} else if (strcmp(*argv, "tos") == 0 ||
				matches(*argv, "dsfield") == 0) {
			__u32 tos;

			NEXT_ARG();
			if (rtnl_dsfield_a2n(&tos, *argv))
				invarg("\"tos\" value is invalid\n", *argv);
			req.r.rtm_tos = tos;
		} else if (strcmp(*argv, "expires") == 0) {
			__u32 expires;

			NEXT_ARG();
			if (get_u32(&expires, *argv, 0))
				invarg("\"expires\" value is invalid\n", *argv);
			if (!hz)
				hz = get_user_hz();
			addattr32(&req.n, sizeof(req), RTA_EXPIRES, expires*hz);
		} else if (matches(*argv, "metric") == 0 ||
				matches(*argv, "priority") == 0 ||
				strcmp(*argv, "preference") == 0) {
			__u32 metric;

			NEXT_ARG();
			if (get_u32(&metric, *argv, 0))
				invarg("\"metric\" value is invalid\n", *argv);
			addattr32(&req.n, sizeof(req), RTA_PRIORITY, metric);
		} else if (strcmp(*argv, "scope") == 0) {
			__u32 scope = 0;

			NEXT_ARG();
			if (rtnl_rtscope_a2n(&scope, *argv))
				invarg("invalid \"scope\" value\n", *argv);
			req.r.rtm_scope = scope;
			scope_ok = 1;
		} else if (strcmp(*argv, "mtu") == 0) {
			unsigned int mtu;

			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= (1<<RTAX_MTU);
				NEXT_ARG();
			}
			if (get_unsigned(&mtu, *argv, 0))
				invarg("\"mtu\" value is invalid\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_MTU, mtu);
		} else if (strcmp(*argv, "hoplimit") == 0) {
			unsigned int hoplimit;

			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= (1<<RTAX_HOPLIMIT);
				NEXT_ARG();
			}
			if (get_unsigned(&hoplimit, *argv, 0) || hoplimit > 255)
				invarg("\"hoplimit\" value is invalid\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_HOPLIMIT, hoplimit);
		} else if (strcmp(*argv, "advmss") == 0) {
			unsigned int mss;

			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= (1<<RTAX_ADVMSS);
				NEXT_ARG();
			}
			if (get_unsigned(&mss, *argv, 0))
				invarg("\"mss\" value is invalid\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_ADVMSS, mss);
		} else if (matches(*argv, "reordering") == 0) {
			unsigned int reord;

			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= (1<<RTAX_REORDERING);
				NEXT_ARG();
			}
			if (get_unsigned(&reord, *argv, 0))
				invarg("\"reordering\" value is invalid\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_REORDERING, reord);
		} else if (strcmp(*argv, "rtt") == 0) {
			unsigned int rtt;

			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= (1<<RTAX_RTT);
				NEXT_ARG();
			}
			if (get_time_rtt(&rtt, *argv, &raw))
				invarg("\"rtt\" value is invalid\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_RTT,
					(raw) ? rtt : rtt * 8);
		} else if (strcmp(*argv, "rto_min") == 0) {
			unsigned int rto_min;

			NEXT_ARG();
			mxlock |= (1<<RTAX_RTO_MIN);
			if (get_time_rtt(&rto_min, *argv, &raw))
				invarg("\"rto_min\" value is invalid\n",
						*argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_RTO_MIN,
					rto_min);
		} else if (matches(*argv, "window") == 0) {
			unsigned int win;

			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= (1<<RTAX_WINDOW);
				NEXT_ARG();
			}
			if (get_unsigned(&win, *argv, 0))
				invarg("\"window\" value is invalid\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_WINDOW, win);
		} else if (matches(*argv, "cwnd") == 0) {
			unsigned int win;

			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= (1<<RTAX_CWND);
				NEXT_ARG();
			}
			if (get_unsigned(&win, *argv, 0))
				invarg("\"cwnd\" value is invalid\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_CWND, win);
		} else if (matches(*argv, "initcwnd") == 0) {
			unsigned int win;

			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= (1<<RTAX_INITCWND);
				NEXT_ARG();
			}
			if (get_unsigned(&win, *argv, 0))
				invarg("\"initcwnd\" value is invalid\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_INITCWND, win);
		} else if (matches(*argv, "initrwnd") == 0) {
			unsigned int win;

			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= (1<<RTAX_INITRWND);
				NEXT_ARG();
			}
			if (get_unsigned(&win, *argv, 0))
				invarg("\"initrwnd\" value is invalid\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_INITRWND, win);
		} else if (matches(*argv, "features") == 0) {
			unsigned int features = 0;

			while (argc > 0) {
				NEXT_ARG();

				if (strcmp(*argv, "ecn") == 0)
					features |= RTAX_FEATURE_ECN;
				else
					invarg("\"features\" value not valid\n", *argv);
				break;
			}

			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_FEATURES, features);
		} else if (matches(*argv, "quickack") == 0) {
			unsigned int quickack;

			NEXT_ARG();
			if (get_unsigned(&quickack, *argv, 0))
				invarg("\"quickack\" value is invalid\n", *argv);
			if (quickack != 1 && quickack != 0)
				invarg("\"quickack\" value should be 0 or 1\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_QUICKACK, quickack);
		} else if (matches(*argv, "congctl") == 0) {
			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= 1 << RTAX_CC_ALGO;
				NEXT_ARG();
			}
			rta_addattr_l(mxrta, sizeof(mxbuf), RTAX_CC_ALGO, *argv,
					strlen(*argv));
		} else if (matches(*argv, "rttvar") == 0) {
			unsigned int win;

			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= (1<<RTAX_RTTVAR);
				NEXT_ARG();
			}
			if (get_time_rtt(&win, *argv, &raw))
				invarg("\"rttvar\" value is invalid\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_RTTVAR,
					(raw) ? win : win * 4);
		} else if (matches(*argv, "ssthresh") == 0) {
			unsigned int win;

			NEXT_ARG();
			if (strcmp(*argv, "lock") == 0) {
				mxlock |= (1<<RTAX_SSTHRESH);
				NEXT_ARG();
			}
			if (get_unsigned(&win, *argv, 0))
				invarg("\"ssthresh\" value is invalid\n", *argv);
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_SSTHRESH, win);
		} else if (matches(*argv, "realms") == 0) {
			__u32 realm;

			NEXT_ARG();
			if (get_rt_realms_or_raw(&realm, *argv))
				invarg("\"realm\" value is invalid\n", *argv);
			addattr32(&req.n, sizeof(req), RTA_FLOW, realm);
		} else if (strcmp(*argv, "onlink") == 0) {
			req.r.rtm_flags |= RTNH_F_ONLINK;
		} else if (strcmp(*argv, "nexthop") == 0) {
			nhs_ok = 1;
			break;
		} else if (matches(*argv, "protocol") == 0) {
			__u32 prot;

			NEXT_ARG();
			if (rtnl_rtprot_a2n(&prot, *argv))
				invarg("\"protocol\" value is invalid\n", *argv);
			req.r.rtm_protocol = prot;
		} else if (matches(*argv, "table") == 0) {
			__u32 tid;

			NEXT_ARG();
			if (rtnl_rttable_a2n(&tid, *argv))
				invarg("\"table\" value is invalid\n", *argv);
			if (tid < 256)
				req.r.rtm_table = tid;
			else {
				req.r.rtm_table = RT_TABLE_UNSPEC;
				addattr32(&req.n, sizeof(req), RTA_TABLE, tid);
			}
			table_ok = 1;
		} else if (strcmp(*argv, "dev") == 0 ||
				strcmp(*argv, "oif") == 0) {
			NEXT_ARG();
			d = *argv;
		} else if (matches(*argv, "pref") == 0) {
			__u8 pref;

			NEXT_ARG();
			if (strcmp(*argv, "low") == 0)
				pref = ICMPV6_ROUTER_PREF_LOW;
			else if (strcmp(*argv, "medium") == 0)
				pref = ICMPV6_ROUTER_PREF_MEDIUM;
			else if (strcmp(*argv, "high") == 0)
				pref = ICMPV6_ROUTER_PREF_HIGH;
			else if (get_u8(&pref, *argv, 0))
				invarg("\"pref\" value is invalid\n", *argv);
			addattr8(&req.n, sizeof(req), RTA_PREF, pref);
		} else if (strcmp(*argv, "encap") == 0) {
			char buf[1024];
			struct rtattr *rta = (void *)buf;

			rta->rta_type = RTA_ENCAP;
			rta->rta_len = RTA_LENGTH(0);

			lwt_parse_encap(rta, sizeof(buf), &argc, &argv);

			if (rta->rta_len > RTA_LENGTH(0))
				addraw_l(&req.n, 1024, RTA_DATA(rta), RTA_PAYLOAD(rta));
		} else {
			int type;
			inet_prefix dst;

			if (strcmp(*argv, "to") == 0) {
				NEXT_ARG();
			}
			if ((**argv < '0' || **argv > '9') &&
					rtnl_rtntype_a2n(&type, *argv) == 0) {
				NEXT_ARG();
				req.r.rtm_type = type;
				type_ok = 1;
			}

			if (matches(*argv, "help") == 0)
				usage();
			if (dst_ok)
				duparg2("to", *argv);
			get_prefix(&dst, *argv, req.r.rtm_family);
			if (req.r.rtm_family == AF_UNSPEC)
				req.r.rtm_family = dst.family;
			req.r.rtm_dst_len = dst.bitlen;
			dst_ok = 1;
			if (dst.bytelen)
				addattr_l(&req.n, sizeof(req), RTA_DST, &dst.data, dst.bytelen);
		}
		argc--; argv++;
	}

	if (!dst_ok)
		usage();

	if (d || nhs_ok)  {
		int idx;

		if (d) {
			if ((idx = ll_name_to_index(d)) == 0) {
				fprintf(stderr, "Cannot find device \"%s\"\n", d);
				return -1;
			}
			addattr32(&req.n, sizeof(req), RTA_OIF, idx);
		}
	}

	if (mxrta->rta_len > RTA_LENGTH(0)) {
		if (mxlock)
			rta_addattr32(mxrta, sizeof(mxbuf), RTAX_LOCK, mxlock);
		addattr_l(&req.n, sizeof(req), RTA_METRICS, RTA_DATA(mxrta), RTA_PAYLOAD(mxrta));
	}

	if (nhs_ok)
		parse_nexthops(&req.n, &req.r, argc, argv);

	if (req.r.rtm_family == AF_UNSPEC)
		req.r.rtm_family = AF_INET;

	if (!table_ok) {
		if (req.r.rtm_type == RTN_LOCAL ||
				req.r.rtm_type == RTN_BROADCAST ||
				req.r.rtm_type == RTN_NAT ||
				req.r.rtm_type == RTN_ANYCAST)
			req.r.rtm_table = RT_TABLE_LOCAL;
	}
	if (!scope_ok) {
		if (req.r.rtm_family == AF_INET6 ||
				req.r.rtm_family == AF_MPLS)
			req.r.rtm_scope = RT_SCOPE_UNIVERSE;
		else if (req.r.rtm_type == RTN_LOCAL ||
				req.r.rtm_type == RTN_NAT)
			req.r.rtm_scope = RT_SCOPE_HOST;
		else if (req.r.rtm_type == RTN_BROADCAST ||
				req.r.rtm_type == RTN_MULTICAST ||
				req.r.rtm_type == RTN_ANYCAST)
			req.r.rtm_scope = RT_SCOPE_LINK;
		else if (req.r.rtm_type == RTN_UNICAST ||
				req.r.rtm_type == RTN_UNSPEC) {
			if (cmd == RTM_DELROUTE)
				req.r.rtm_scope = RT_SCOPE_NOWHERE;
			else if (!gw_ok && !nhs_ok)
				req.r.rtm_scope = RT_SCOPE_LINK;
		}
	}

	if (!type_ok && req.r.rtm_family == AF_MPLS)
		req.r.rtm_type = RTN_UNICAST;

	if (rtnl_talk(&rth, &req.n, NULL, 0) < 0)
		return -2;

	return 0;
}
#endif
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
static void
send_eth_frame(void)
{
	char buf[BUF_SIZE];
	struct ifreq ifr;
	struct ifreq ifr_mac;
	struct sockaddr_ll socket_address;
	int tx_len = 0;
	struct ether_header *eh;
	int s;

    	s = socket(AF_PACKET, SOCK_RAW, IPPROTO_TCP);
	if (s == -1) {
		perror("socket");
		return;
	}

	/* Get the index of the interface to send on. */
	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_name, "vEth1_0", IFNAMSIZ - 1);
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
		perror("ioctl SIOCGIFINDEX");

	/* Get the MAC address of the interface to send on. */
	memset(&ifr_mac, 0, sizeof(struct ifreq));
	strncpy(ifr_mac.ifr_name, "vEth1_0", IFNAMSIZ - 1);
	if (ioctl(s, SIOCGIFHWADDR, &ifr_mac) < 0)
		perror("ioctl SIOCGIFHWADDR");

	/* Construct the Ethernet header */
	memset(buf, 0, BUF_SIZE);
	/* Ethernet header */
	eh = (struct ether_header *)buf;
	memcpy(eh->ether_shost, ifr_mac.ifr_hwaddr.sa_data, ETH_ALEN);
	memset(eh->ether_dhost, 0xff, ETH_ALEN);

	/* Ethertype field */
	eh->ether_type = htons(ETH_P_EXPERIMENTAL);
	tx_len += sizeof(struct ether_header);

	/* Packet data */
	buf[tx_len++] = 'a';
	buf[tx_len++] = 'b';
	buf[tx_len++] = 'c';
	buf[tx_len++] = 'd';

	/* Index of the network device */
	socket_address.sll_ifindex = ifr.ifr_ifindex;
	/* Address length*/
	socket_address.sll_halen = ETH_ALEN;
	/* Destination MAC */
	memset(socket_address.sll_addr, 0xff, ETH_ALEN);

	/* Send packet */
	if (sendto(s, buf, tx_len, 0, (struct sockaddr*)&socket_address,
		   sizeof(struct sockaddr_ll)) < 0)
	    perror("sendto\n");
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
