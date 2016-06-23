#ifndef __BGP_NL_H__
#define __BGP_NL_H__

#include <linux/rtnetlink.h>

#define ETHER_TYPE_EXPER	0x88B5

int handle_nlmsg(const struct nlmsghdr *nlh);

#endif /* __BGP_NL_H__ */
