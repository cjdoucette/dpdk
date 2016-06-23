#ifndef __BGP_COMMON_H__
#define __BGP_COMMON_H__

#include <rte_ether.h>

/* Macros for printing using RTE_LOG */
#define RTE_LOGTYPE_APP RTE_LOGTYPE_USER1

/* Max size of a single packet */
#define MAX_PACKET_SZ           2048

/* Options for configuring ethernet port */
extern struct rte_eth_conf port_conf;

extern struct rte_mempool *pktmbuf_pool;

/* Mask of enabled ports */
extern uint32_t ports_mask;

/* Ports set in promiscuous mode on by default. */
extern int promiscuous_on;

extern uint64_t dest_eth_addr[RTE_MAX_ETHPORTS];
extern struct ether_addr ports_eth_addr[RTE_MAX_ETHPORTS];

#endif /* __BGP_COMMON_H__ */
