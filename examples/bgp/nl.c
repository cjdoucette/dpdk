#include <libmnl/libmnl.h>
#include <stdio.h>
#include <arpa/inet.h>

#include "nl.h"
#include "fib.h"

/*
 * XXX DPDK LPM only supports egress interface, not gateway.
 * What to do with RTA_GATEWAY?
 */
static void
add_to_fib(struct nlattr *tb[], uint16_t type, uint16_t flags, uint8_t prefix)
{
	struct in_addr *dst;
	uint8_t oif;

	if (!tb[RTA_DST]) {
		printf("cannot add or delete entry without RTA_DST\n");
		return;
	}

	if (prefix > 32) {
		printf("prefix must be in the range [0, 32]\n");
		return;
	}

	dst = mnl_attr_get_payload(tb[RTA_DST]);

	switch (type) {
	case RTM_NEWROUTE:
		if (!tb[RTA_OIF]) {
			printf("cannot add/change route without RTA_OIF\n");
			return;
		}

		printf("flags: %4hx\n", flags);
		if (flags == (NLM_F_REQUEST | NLM_F_CREATE | NLM_F_EXCL)) {
			oif = mnl_attr_get_u8(tb[RTA_OIF]);
			fib_add_route(dst->s_addr, prefix, oif);
		} else if (flags == (NLM_F_REQUEST | NLM_F_REPLACE)) {
			oif = mnl_attr_get_u8(tb[RTA_OIF]);
			fib_add_route(dst->s_addr, prefix, oif);
		} else
			printf("incorrect flags for RTM_NEWROUTE\n");

		break;
	case RTM_DELROUTE:
		fib_del_route(dst->s_addr, prefix);
		break;
	default:
		break;
	}
}

static void
attributes_show_ipv4(struct nlattr *tb[])
{
	if (tb[RTA_TABLE]) {
		printf("table=%u ", mnl_attr_get_u32(tb[RTA_TABLE]));
	}
	if (tb[RTA_DST]) {
		struct in_addr *addr = mnl_attr_get_payload(tb[RTA_DST]);
		printf("dst=%s ", inet_ntoa(*addr));
	}
	if (tb[RTA_SRC]) {
		struct in_addr *addr = mnl_attr_get_payload(tb[RTA_SRC]);
		printf("src=%s ", inet_ntoa(*addr));
	}
	if (tb[RTA_OIF]) {
		printf("oif=%u ", mnl_attr_get_u32(tb[RTA_OIF]));
	}
	if (tb[RTA_FLOW]) {
		printf("flow=%u ", mnl_attr_get_u32(tb[RTA_FLOW]));
	}
	if (tb[RTA_PREFSRC]) {
		struct in_addr *addr = mnl_attr_get_payload(tb[RTA_PREFSRC]);
		printf("prefsrc=%s ", inet_ntoa(*addr));
	}
	if (tb[RTA_GATEWAY]) {
		struct in_addr *addr = mnl_attr_get_payload(tb[RTA_GATEWAY]);
		printf("gw=%s ", inet_ntoa(*addr));
	}
	if (tb[RTA_PRIORITY]) {
		printf("prio=%u ", mnl_attr_get_u32(tb[RTA_PRIORITY]));
	}
	printf("\n");
}

static int
data_ipv4_attr_cb(const struct nlattr *attr, void *data)
{
	const struct nlattr **tb = data;
	int type = mnl_attr_get_type(attr);

	/* skip unsupported attribute in user-space */
	if (mnl_attr_type_valid(attr, RTA_MAX) < 0)
		return MNL_CB_OK;

	switch(type) {
	case RTA_TABLE:
	case RTA_DST:
	case RTA_SRC:
	case RTA_OIF:
	case RTA_FLOW:
	case RTA_PREFSRC:
	case RTA_GATEWAY:
	case RTA_PRIORITY:
		if (mnl_attr_validate(attr, MNL_TYPE_U32) < 0) {
			perror("mnl_attr_validate");
			return MNL_CB_ERROR;
		}
		break;
	case RTA_METRICS:
		if (mnl_attr_validate(attr, MNL_TYPE_NESTED) < 0) {
			perror("mnl_attr_validate");
			return MNL_CB_ERROR;
		}
		break;
	}
	tb[type] = attr;
	return MNL_CB_OK;
}

int
handle_nlmsg(const struct nlmsghdr *nlh)
{
	struct nlattr *tb[RTA_MAX + 1] = {};
	struct rtmsg *rm = mnl_nlmsg_get_payload(nlh);

	switch(nlh->nlmsg_type) {
	case RTM_NEWROUTE:
		printf("[NEW] ");
		break;
	case RTM_DELROUTE:
		printf("[DEL] ");
		break;
	}

	/* protocol family = AF_INET */
	printf("family=%u ", rm->rtm_family);

	/* destination CIDR, eg. 24 or 32 for IPv4 */
	printf("dst_len=%u ", rm->rtm_dst_len);

	/* source CIDR */
	printf("src_len=%u ", rm->rtm_src_len);

	/* type of service (TOS), eg. 0 */
	printf("tos=%u ", rm->rtm_tos);

	/* table id:
	 *	RT_TABLE_UNSPEC		= 0
	 *
	 * 	... user defined values ...
	 *
	 *	RT_TABLE_COMPAT		= 252
	 *	RT_TABLE_DEFAULT	= 253
	 *	RT_TABLE_MAIN		= 254
	 *	RT_TABLE_LOCAL		= 255
	 *	RT_TABLE_MAX		= 0xFFFFFFFF
	 *
	 * Synonimous attribute: RTA_TABLE.
	 */
	printf("table=%u ", rm->rtm_table);

	/* type:
	 * 	RTN_UNSPEC	= 0
	 * 	RTN_UNICAST	= 1
	 * 	RTN_LOCAL	= 2
	 * 	RTN_BROADCAST	= 3
	 *	RTN_ANYCAST	= 4
	 *	RTN_MULTICAST	= 5
	 *	RTN_BLACKHOLE	= 6
	 *	RTN_UNREACHABLE	= 7
	 *	RTN_PROHIBIT	= 8
	 *	RTN_THROW	= 9
	 *	RTN_NAT		= 10
	 *	RTN_XRESOLVE	= 11
	 *	__RTN_MAX	= 12
	 */
	printf("type=%u ", rm->rtm_type);

	/* scope:
	 * 	RT_SCOPE_UNIVERSE	= 0   : everywhere in the universe
	 *
	 *      ... user defined values ...
	 *
	 * 	RT_SCOPE_SITE		= 200
	 * 	RT_SCOPE_LINK		= 253 : destination attached to link
	 * 	RT_SCOPE_HOST		= 254 : local address
	 * 	RT_SCOPE_NOWHERE	= 255 : not existing destination
	 */
	printf("scope=%u ", rm->rtm_scope);

	/* protocol:
	 * 	RTPROT_UNSPEC	= 0
	 * 	RTPROT_REDIRECT = 1
	 * 	RTPROT_KERNEL	= 2 : route installed by kernel
	 * 	RTPROT_BOOT	= 3 : route installed during boot
	 * 	RTPROT_STATIC	= 4 : route installed by administrator
	 *
	 * Values >= RTPROT_STATIC are not interpreted by kernel, they are
	 * just user-defined.
	 */
	printf("proto=%u ", rm->rtm_protocol);

	/* flags:
	 * 	RTM_F_NOTIFY	= 0x100: notify user of route change
	 * 	RTM_F_CLONED	= 0x200: this route is cloned
	 * 	RTM_F_EQUALIZE	= 0x400: Multipath equalizer: NI
	 * 	RTM_F_PREFIX	= 0x800: Prefix addresses
	 */
	printf("flags=%x ", rm->rtm_flags);

	switch(rm->rtm_family) {
	case AF_INET:
		mnl_attr_parse(nlh, sizeof(*rm), data_ipv4_attr_cb, tb);
		attributes_show_ipv4(tb);
		add_to_fib(tb, nlh->nlmsg_type, nlh->nlmsg_flags,
			   rm->rtm_dst_len);
		break;
	}

	printf("\n");
	return MNL_CB_OK;
}
