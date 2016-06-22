#ifndef __BGP_KNI_H__
#define __BGP_KNI_H__

#define KNI_MAX_KTHREAD 32

/*
 * Structure of port parameters
 */
struct kni_port_params {
	uint8_t port_id;/* Port ID */
	unsigned lcore_rx; /* lcore ID for RX */
	unsigned lcore_tx; /* lcore ID for TX */
	uint32_t nb_lcore_k; /* Number of lcores for KNI multi kernel threads */
	uint32_t nb_kni; /* Number of KNI devices to be created */
	unsigned lcore_k[KNI_MAX_KTHREAD]; /* lcore ID list for kthreads */
	struct rte_kni *kni[KNI_MAX_KTHREAD]; /* KNI context pointers */
} __rte_cache_aligned;

extern struct kni_port_params *kni_port_params_array[RTE_MAX_ETHPORTS];

void init_kni(void);
void kni_release(uint8_t nb_sys_ports);
int kni_alloc(uint8_t port_id);

#endif /* __BGP_KNI_H__ */
