/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2010-2014 Intel Corporation
 */

#ifndef _RTE_LPM_H_
#define _RTE_LPM_H_

/**
 * @file
 * RTE Longest Prefix Match (LPM)
 */

#include <errno.h>
#include <sys/queue.h>
#include <stdint.h>
#include <stdlib.h>
#include <rte_prefetch.h>
#include <rte_branch_prediction.h>
#include <rte_byteorder.h>
#include <rte_config.h>
#include <rte_memory.h>
#include <rte_common.h>
#include <rte_vect.h>
#include <rte_compat.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Max number of characters in LPM name. */
#define RTE_LPM_NAMESIZE                32

/** Maximum depth value possible for IPv4 LPM. */
#define RTE_LPM_MAX_DEPTH               32

/** @internal Total number of tbl24 entries. */
#define RTE_LPM_TBL24_NUM_ENTRIES       (1 << 24)

/** @internal Number of entries in a tbl8 group. */
#define RTE_LPM_TBL8_GROUP_NUM_ENTRIES  256

/** @internal Max number of tbl8 groups in the tbl8. */
#define RTE_LPM_MAX_TBL8_NUM_GROUPS         (1 << 24)

/** @internal Total number of tbl8 groups in the tbl8. */
#define RTE_LPM_TBL8_NUM_GROUPS         256

/** @internal Total number of tbl8 entries. */
#define RTE_LPM_TBL8_NUM_ENTRIES        (RTE_LPM_TBL8_NUM_GROUPS * \
					RTE_LPM_TBL8_GROUP_NUM_ENTRIES)

/** @internal Macro to enable/disable run-time checks. */
#if defined(RTE_LIBRTE_LPM_DEBUG)
#define RTE_LPM_RETURN_IF_TRUE(cond, retval) do { \
	if (cond) return (retval);                \
} while (0)
#else
#define RTE_LPM_RETURN_IF_TRUE(cond, retval)
#endif

/** @internal bitmask with valid and valid_group fields set */
#define RTE_LPM_VALID_EXT_ENTRY_BITMASK 0x03000000

/** Bitmask used to indicate successful lookup */
#define RTE_LPM_LOOKUP_SUCCESS          0x01000000

#if RTE_BYTE_ORDER == RTE_LITTLE_ENDIAN
/** @internal Tbl24 entry structure. */
__extension__
struct rte_lpm_tbl_entry_v20 {
	/**
	 * Stores Next hop (tbl8 or tbl24 when valid_group is not set) or
	 * a group index pointing to a tbl8 structure (tbl24 only, when
	 * valid_group is set)
	 */
	RTE_STD_C11
	union {
		uint8_t next_hop;
		uint8_t group_idx;
	};
	/* Using single uint8_t to store 3 values. */
	uint8_t valid     :1;   /**< Validation flag. */
	/**
	 * For tbl24:
	 *  - valid_group == 0: entry stores a next hop
	 *  - valid_group == 1: entry stores a group_index pointing to a tbl8
	 * For tbl8:
	 *  - valid_group indicates whether the current tbl8 is in use or not
	 */
	uint8_t valid_group :1;
	uint8_t depth       :6; /**< Rule depth. */
} __rte_aligned(sizeof(uint16_t));

__extension__
struct rte_lpm_tbl_entry {
	/**
	 * Stores Next hop (tbl8 or tbl24 when valid_group is not set) or
	 * a group index pointing to a tbl8 structure (tbl24 only, when
	 * valid_group is set)
	 */
	uint32_t next_hop    :24;
	/* Using single uint8_t to store 3 values. */
	uint32_t valid       :1;   /**< Validation flag. */
	/**
	 * For tbl24:
	 *  - valid_group == 0: entry stores a next hop
	 *  - valid_group == 1: entry stores a group_index pointing to a tbl8
	 * For tbl8:
	 *  - valid_group indicates whether the current tbl8 is in use or not
	 */
	uint32_t valid_group :1;
	uint32_t depth       :6; /**< Rule depth. */
};

#else
__extension__
struct rte_lpm_tbl_entry_v20 {
	uint8_t depth       :6;
	uint8_t valid_group :1;
	uint8_t valid       :1;
	union {
		uint8_t group_idx;
		uint8_t next_hop;
	};
} __rte_aligned(sizeof(uint16_t));

__extension__
struct rte_lpm_tbl_entry {
	uint32_t depth       :6;
	uint32_t valid_group :1;
	uint32_t valid       :1;
	uint32_t next_hop    :24;

};

#endif

/** LPM configuration structure. */
struct rte_lpm_config {
	uint32_t max_rules;      /**< Max number of rules. */
	uint32_t number_tbl8s;   /**< Number of tbl8s to allocate. */
	int flags;               /**< This field is currently unused. */
};

/** @internal Rule structure. */
struct rte_lpm_rule_v20 {
	uint32_t ip; /**< Rule IP address. */
	uint8_t  next_hop; /**< Rule next hop. */
};

struct rte_lpm_rule {
	uint32_t ip; /**< Rule IP address. */
	uint32_t next_hop; /**< Rule next hop. */
};

/** @internal Contains metadata about the rules table. */
struct rte_lpm_rule_info {
	uint32_t used_rules; /**< Used rules so far. */
	uint32_t first_rule; /**< Indexes the first rule of a given depth. */
};

/** @internal LPM structure. */
struct rte_lpm_v20 {
	/* LPM metadata. */
	char name[RTE_LPM_NAMESIZE];        /**< Name of the lpm. */
	uint32_t max_rules; /**< Max. balanced rules per lpm. */
	struct rte_lpm_rule_info rule_info[RTE_LPM_MAX_DEPTH]; /**< Rule info table. */

	/* LPM Tables. */
	struct rte_lpm_tbl_entry_v20 tbl24[RTE_LPM_TBL24_NUM_ENTRIES]
			__rte_cache_aligned; /**< LPM tbl24 table. */
	struct rte_lpm_tbl_entry_v20 tbl8[RTE_LPM_TBL8_NUM_ENTRIES]
			__rte_cache_aligned; /**< LPM tbl8 table. */
	struct rte_lpm_rule_v20 rules_tbl[]
			__rte_cache_aligned; /**< LPM rules. */
};

struct rte_lpm {
	/* LPM metadata. */
	char name[RTE_LPM_NAMESIZE];        /**< Name of the lpm. */
	uint32_t max_rules; /**< Max. balanced rules per lpm. */
	uint32_t number_tbl8s; /**< Number of tbl8s. */
	struct rte_lpm_rule_info rule_info[RTE_LPM_MAX_DEPTH]; /**< Rule info table. */

	/* LPM Tables. */
	struct rte_lpm_tbl_entry tbl24[RTE_LPM_TBL24_NUM_ENTRIES]
			__rte_cache_aligned; /**< LPM tbl24 table. */
	struct rte_lpm_tbl_entry *tbl8; /**< LPM tbl8 table. */
	struct rte_lpm_rule *rules_tbl; /**< LPM rules. */
};

/** LPM iterator state structure. */
struct rte_lpm_iterator_state {
	uint32_t dmask;
	uint32_t ip_masked;
	uint8_t  depth;
	uint32_t next;
	const struct rte_lpm *lpm;
};

/**
 * Initialize the lpm iterator state.
 *
 * @param lpm
 *   LPM object handle
 * @param ip
 *   IP of the rule to be searched
 * @param depth
 *   Initial depth of the rule to be searched.
 *   Pass zero to enumerate the whole LPM table.
 * @param state
 *   Pointer to the iterator state
 * @return
 *   0 on successfully initialize the state variable, negative otherwise.
 *   Possible error values include:
 *   - EINVAL - invalid parameter passed to function
 */
int
rte_lpm_iterator_state_init(const struct rte_lpm *lpm, uint32_t ip,
	uint8_t depth, struct rte_lpm_iterator_state *state);

/**
 * An iterator over its rule entries.
 * The iterator should require a prefix as a parameter
 * and should list all entries as long as the given prefix (or longer).
 *
 * @param state
 *   Pointer to the LPM rule iterator state
 * @param rule
 *   Pointer to the next rule entry
 * @return
 *   0 on successfully searching the next rule entry, negative otherwise.
 *   Possible error values include:
 *   - EINVAL - invalid parameter passed to function
 *   - ENOENT - no rule entries found
 */
int
rte_lpm_rule_iterate(struct rte_lpm_iterator_state *state,
	const struct rte_lpm_rule **rule);
int
rte_lpm_rule_iterate_v20(struct rte_lpm_iterator_state *state,
	const struct rte_lpm_rule **rule);
int
rte_lpm_rule_iterate_v1604(struct rte_lpm_iterator_state *state,
	const struct rte_lpm_rule **rule);

/**
 * Create an LPM object.
 *
 * @param name
 *   LPM object name
 * @param socket_id
 *   NUMA socket ID for LPM table memory allocation
 * @param config
 *   Structure containing the configuration
 * @return
 *   Handle to LPM object on success, NULL otherwise with rte_errno set
 *   to an appropriate values. Possible rte_errno values include:
 *    - E_RTE_NO_CONFIG - function could not get pointer to rte_config structure
 *    - E_RTE_SECONDARY - function was called from a secondary process instance
 *    - EINVAL - invalid parameter passed to function
 *    - ENOSPC - the maximum number of memzones has already been allocated
 *    - EEXIST - a memzone with the same name already exists
 *    - ENOMEM - no appropriate memory area found in which to create memzone
 */
struct rte_lpm *
rte_lpm_create(const char *name, int socket_id,
		const struct rte_lpm_config *config);
struct rte_lpm_v20 *
rte_lpm_create_v20(const char *name, int socket_id, int max_rules, int flags);
struct rte_lpm *
rte_lpm_create_v1604(const char *name, int socket_id,
		const struct rte_lpm_config *config);

/**
 * Find an existing LPM object and return a pointer to it.
 *
 * @param name
 *   Name of the lpm object as passed to rte_lpm_create()
 * @return
 *   Pointer to lpm object or NULL if object not found with rte_errno
 *   set appropriately. Possible rte_errno values include:
 *    - ENOENT - required entry not available to return.
 */
struct rte_lpm *
rte_lpm_find_existing(const char *name);
struct rte_lpm_v20 *
rte_lpm_find_existing_v20(const char *name);
struct rte_lpm *
rte_lpm_find_existing_v1604(const char *name);

/**
 * Free an LPM object.
 *
 * @param lpm
 *   LPM object handle
 * @return
 *   None
 */
void
rte_lpm_free(struct rte_lpm *lpm);
void
rte_lpm_free_v20(struct rte_lpm_v20 *lpm);
void
rte_lpm_free_v1604(struct rte_lpm *lpm);

/**
 * Add a rule to the LPM table.
 *
 * @param lpm
 *   LPM object handle
 * @param ip
 *   IP of the rule to be added to the LPM table
 * @param depth
 *   Depth of the rule to be added to the LPM table
 * @param next_hop
 *   Next hop of the rule to be added to the LPM table
 * @return
 *   0 on success, negative value otherwise
 */
int
rte_lpm_add(struct rte_lpm *lpm, uint32_t ip, uint8_t depth, uint32_t next_hop);
int
rte_lpm_add_v20(struct rte_lpm_v20 *lpm, uint32_t ip, uint8_t depth,
		uint8_t next_hop);
int
rte_lpm_add_v1604(struct rte_lpm *lpm, uint32_t ip, uint8_t depth,
		uint32_t next_hop);

/**
 * Check if a rule is present in the LPM table,
 * and provide its next hop if it is.
 *
 * @param lpm
 *   LPM object handle
 * @param ip
 *   IP of the rule to be searched
 * @param depth
 *   Depth of the rule to searched
 * @param next_hop
 *   Next hop of the rule (valid only if it is found)
 * @return
 *   1 if the rule exists, 0 if it does not, a negative value on failure
 */
int
rte_lpm_is_rule_present(struct rte_lpm *lpm, uint32_t ip, uint8_t depth,
uint32_t *next_hop);
int
rte_lpm_is_rule_present_v20(struct rte_lpm_v20 *lpm, uint32_t ip, uint8_t depth,
uint8_t *next_hop);
int
rte_lpm_is_rule_present_v1604(struct rte_lpm *lpm, uint32_t ip, uint8_t depth,
uint32_t *next_hop);

/**
 * Delete a rule from the LPM table.
 *
 * @param lpm
 *   LPM object handle
 * @param ip
 *   IP of the rule to be deleted from the LPM table
 * @param depth
 *   Depth of the rule to be deleted from the LPM table
 * @return
 *   0 on success, negative value otherwise
 */
int
rte_lpm_delete(struct rte_lpm *lpm, uint32_t ip, uint8_t depth);
int
rte_lpm_delete_v20(struct rte_lpm_v20 *lpm, uint32_t ip, uint8_t depth);
int
rte_lpm_delete_v1604(struct rte_lpm *lpm, uint32_t ip, uint8_t depth);

/**
 * Delete all rules from the LPM table.
 *
 * @param lpm
 *   LPM object handle
 */
void
rte_lpm_delete_all(struct rte_lpm *lpm);
void
rte_lpm_delete_all_v20(struct rte_lpm_v20 *lpm);
void
rte_lpm_delete_all_v1604(struct rte_lpm *lpm);

/**
 * Lookup an IP into the LPM table.
 *
 * @param lpm
 *   LPM object handle
 * @param ip
 *   IP to be looked up in the LPM table
 * @param next_hop
 *   Next hop of the most specific rule found for IP (valid on lookup hit only)
 * @return
 *   -EINVAL for incorrect arguments, -ENOENT on lookup miss, 0 on lookup hit
 */
static inline int
rte_lpm_lookup(struct rte_lpm *lpm, uint32_t ip, uint32_t *next_hop)
{
	unsigned tbl24_index = (ip >> 8);
	uint32_t tbl_entry;
	const uint32_t *ptbl;

	/* DEBUG: Check user input arguments. */
	RTE_LPM_RETURN_IF_TRUE(((lpm == NULL) || (next_hop == NULL)), -EINVAL);

	/* Copy tbl24 entry */
	ptbl = (const uint32_t *)(&lpm->tbl24[tbl24_index]);
	tbl_entry = *ptbl;

	/* Memory ordering is not required in lookup. Because dataflow
	 * dependency exists, compiler or HW won't be able to re-order
	 * the operations.
	 */
	/* Copy tbl8 entry (only if needed) */
	if (unlikely((tbl_entry & RTE_LPM_VALID_EXT_ENTRY_BITMASK) ==
			RTE_LPM_VALID_EXT_ENTRY_BITMASK)) {

		unsigned tbl8_index = (uint8_t)ip +
				(((uint32_t)tbl_entry & 0x00FFFFFF) *
						RTE_LPM_TBL8_GROUP_NUM_ENTRIES);

		ptbl = (const uint32_t *)&lpm->tbl8[tbl8_index];
		tbl_entry = *ptbl;
	}

	*next_hop = ((uint32_t)tbl_entry & 0x00FFFFFF);
	return (tbl_entry & RTE_LPM_LOOKUP_SUCCESS) ? 0 : -ENOENT;
}

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Prefetch the tbl24 entry into all cache levels.
 *
 * @param lpm
 *   LPM object handle
 * @param ip
 *   IP to be looked up in the LPM table
 */
__rte_experimental
static inline void
rte_lpm_prefetch_tbl24_entry(struct rte_lpm *lpm, uint32_t ip)
{
	rte_prefetch0(&lpm->tbl24[(ip >> 8)]);
}

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Prefetch the tbl8 entry into all cache levels.
 *
 * @param lpm
 *   LPM object handle
 * @param tbl8_index
 *   Index of the tbl8 entry
 */
__rte_experimental
static inline void
rte_lpm_prefetch_tbl8_entry(struct rte_lpm *lpm, unsigned tbl8_index)
{
	rte_prefetch0(&lpm->tbl8[tbl8_index]);
}

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Prefetch the tbl24 entry into all cache levels
 * (non-temporal/transient version).
 *
 * @param lpm
 *   LPM object handle
 * @param ip
 *   IP to be looked up in the LPM table
 */
__rte_experimental
static inline void
rte_lpm_prefetch_tbl24_entry_non_temporal(struct rte_lpm *lpm, uint32_t ip)
{
	rte_prefetch_non_temporal(&lpm->tbl24[(ip >> 8)]);
}

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Prefetch the tbl8 entry into all cache levels
 * (non-temporal/transient version).
 *
 * @param lpm
 *   LPM object handle
 * @param tbl8_index
 *   Index of the tbl8 entry
 */
__rte_experimental
static inline void
rte_lpm_prefetch_tbl8_entry_non_temporal(struct rte_lpm *lpm,
	unsigned tbl8_index)
{
	rte_prefetch_non_temporal(&lpm->tbl8[tbl8_index]);
}

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Lookup an IP into the LPM table in the first level.
 *
 * @param lpm
 *   LPM object handle
 * @param tbl8_index
 *   Pointer to the index of the tbl8 table
 * @param ip
 *   IP to be looked up in the LPM table
 * @param next_hop
 *   Next hop of the most specific rule found for IP (valid on lookup hit only)
 * @return
 *   -EINVAL for incorrect arguments, -ENOENT on lookup miss, 0 on lookup hit,
 *   1 if the process needs to be continued by calling the function
 *   rte_lpm_lookup_step2()
 */
__rte_experimental
static inline int
rte_lpm_lookup_step1(struct rte_lpm *lpm, unsigned *tbl8_index,
	uint32_t ip, uint32_t *next_hop)
{
	unsigned tbl24_index = (ip >> 8);
	uint32_t tbl_entry;
	const uint32_t *ptbl;

	/* DEBUG: Check user input arguments. */
	RTE_LPM_RETURN_IF_TRUE(((lpm == NULL) || (next_hop == NULL)), -EINVAL);

	/* Copy tbl24 entry */
	ptbl = (const uint32_t *)(&lpm->tbl24[tbl24_index]);
	tbl_entry = *ptbl;

	/* Memory ordering is not required in lookup. Because dataflow
	 * dependency exists, compiler or HW won't be able to re-order
	 * the operations.
	 */
	/* Copy tbl8 entry (only if needed) */
	if (unlikely((tbl_entry & RTE_LPM_VALID_EXT_ENTRY_BITMASK) ==
			RTE_LPM_VALID_EXT_ENTRY_BITMASK)) {

		*tbl8_index = (uint8_t)ip +
				(((uint32_t)tbl_entry & 0x00FFFFFF) *
						RTE_LPM_TBL8_GROUP_NUM_ENTRIES);

		return 1;
	}

	*next_hop = ((uint32_t)tbl_entry & 0x00FFFFFF);
	return (tbl_entry & RTE_LPM_LOOKUP_SUCCESS) ? 0 : -ENOENT;
}

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Lookup an IP into the LPM table in the second level.
 *
 * @param lpm
 *   LPM object handle
 * @param tbl8_index
 *   Index to the tbl8 table
 * @param next_hop
 *   Next hop of the most specific rule found for IP (valid on lookup hit only)
 * @return
 *   -EINVAL for incorrect arguments, -ENOENT on lookup miss, 0 on lookup hit
 */
__rte_experimental
static inline int
rte_lpm_lookup_step2(struct rte_lpm *lpm, unsigned tbl8_index,
	uint32_t *next_hop)
{
	uint32_t tbl_entry;
	const uint32_t *ptbl;

	/* DEBUG: Check user input arguments. */
	RTE_LPM_RETURN_IF_TRUE(((lpm == NULL) || (next_hop == NULL)), -EINVAL);

	ptbl = (const uint32_t *)&lpm->tbl8[tbl8_index];
	tbl_entry = *ptbl;

	*next_hop = ((uint32_t)tbl_entry & 0x00FFFFFF);
	return (tbl_entry & RTE_LPM_LOOKUP_SUCCESS) ? 0 : -ENOENT;
}

typedef void (*rte_hash_yield_func)(void *addr, void *arg);

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Lookup an IP into the LPM table and yield.
 *
 * @param lpm
 *   LPM object handle
 * @param ip
 *   IP to be looked up in the LPM table
 * @param next_hop
 *   Next hop of the most specific rule found for IP (valid on lookup hit only)
 * @param yield_func
 *   Function used to prefetch and then yield.
 * @param arg
 *   Parameter passed to the function yield_func.
 * @return
 *   -EINVAL for incorrect arguments, -ENOENT on lookup miss, 0 on lookup hit
 */
__rte_experimental
static inline int
rte_lpm_lookup_and_yield(struct rte_lpm *lpm, uint32_t ip, uint32_t *next_hop,
	rte_hash_yield_func yield_func, void *arg)
{
	unsigned tbl24_index = (ip >> 8);
	uint32_t tbl_entry;
	const uint32_t *ptbl;

	/* DEBUG: Check user input arguments. */
	RTE_LPM_RETURN_IF_TRUE(((lpm == NULL) || (next_hop == NULL)), -EINVAL);

	yield_func(&lpm->tbl24[tbl24_index], arg);
	/* Copy tbl24 entry */
	ptbl = (const uint32_t *)(&lpm->tbl24[tbl24_index]);
	tbl_entry = *ptbl;

	/* Memory ordering is not required in lookup. Because dataflow
	 * dependency exists, compiler or HW won't be able to re-order
	 * the operations.
	 */
	/* Copy tbl8 entry (only if needed) */
	if (unlikely((tbl_entry & RTE_LPM_VALID_EXT_ENTRY_BITMASK) ==
			RTE_LPM_VALID_EXT_ENTRY_BITMASK)) {

		unsigned tbl8_index = (uint8_t)ip +
				(((uint32_t)tbl_entry & 0x00FFFFFF) *
						RTE_LPM_TBL8_GROUP_NUM_ENTRIES);

		yield_func(&lpm->tbl8[tbl8_index], arg);
		ptbl = (const uint32_t *)&lpm->tbl8[tbl8_index];
		tbl_entry = *ptbl;
	}

	*next_hop = ((uint32_t)tbl_entry & 0x00FFFFFF);
	return (tbl_entry & RTE_LPM_LOOKUP_SUCCESS) ? 0 : -ENOENT;
}

/**
 * Lookup multiple IP addresses in an LPM table. This may be implemented as a
 * macro, so the address of the function should not be used.
 *
 * @param lpm
 *   LPM object handle
 * @param ips
 *   Array of IPs to be looked up in the LPM table
 * @param next_hops
 *   Next hop of the most specific rule found for IP (valid on lookup hit only).
 *   This is an array of two byte values. The most significant byte in each
 *   value says whether the lookup was successful (bitmask
 *   RTE_LPM_LOOKUP_SUCCESS is set). The least significant byte is the
 *   actual next hop.
 * @param n
 *   Number of elements in ips (and next_hops) array to lookup. This should be a
 *   compile time constant, and divisible by 8 for best performance.
 *  @return
 *   -EINVAL for incorrect arguments, otherwise 0
 */
#define rte_lpm_lookup_bulk(lpm, ips, next_hops, n) \
		rte_lpm_lookup_bulk_func(lpm, ips, next_hops, n)

static inline int
rte_lpm_lookup_bulk_func(const struct rte_lpm *lpm, const uint32_t *ips,
		uint32_t *next_hops, const unsigned n)
{
	unsigned i;
	unsigned tbl24_indexes[n];
	const uint32_t *ptbl;

	/* DEBUG: Check user input arguments. */
	RTE_LPM_RETURN_IF_TRUE(((lpm == NULL) || (ips == NULL) ||
			(next_hops == NULL)), -EINVAL);

	for (i = 0; i < n; i++) {
		tbl24_indexes[i] = ips[i] >> 8;
	}

	for (i = 0; i < n; i++) {
		/* Simply copy tbl24 entry to output */
		ptbl = (const uint32_t *)&lpm->tbl24[tbl24_indexes[i]];
		next_hops[i] = *ptbl;

		/* Overwrite output with tbl8 entry if needed */
		if (unlikely((next_hops[i] & RTE_LPM_VALID_EXT_ENTRY_BITMASK) ==
				RTE_LPM_VALID_EXT_ENTRY_BITMASK)) {

			unsigned tbl8_index = (uint8_t)ips[i] +
					(((uint32_t)next_hops[i] & 0x00FFFFFF) *
					 RTE_LPM_TBL8_GROUP_NUM_ENTRIES);

			ptbl = (const uint32_t *)&lpm->tbl8[tbl8_index];
			next_hops[i] = *ptbl;
		}
	}
	return 0;
}

/* Mask four results. */
#define	 RTE_LPM_MASKX4_RES	UINT64_C(0x00ffffff00ffffff)

/**
 * Lookup four IP addresses in an LPM table.
 *
 * @param lpm
 *   LPM object handle
 * @param ip
 *   Four IPs to be looked up in the LPM table
 * @param hop
 *   Next hop of the most specific rule found for IP (valid on lookup hit only).
 *   This is an 4 elements array of two byte values.
 *   If the lookup was successful for the given IP, then least significant byte
 *   of the corresponding element is the  actual next hop and the most
 *   significant byte is zero.
 *   If the lookup for the given IP failed, then corresponding element would
 *   contain default value, see description of then next parameter.
 * @param defv
 *   Default value to populate into corresponding element of hop[] array,
 *   if lookup would fail.
 */
static inline void
rte_lpm_lookupx4(const struct rte_lpm *lpm, xmm_t ip, uint32_t hop[4],
	uint32_t defv);

#if defined(RTE_ARCH_ARM) || defined(RTE_ARCH_ARM64)
#include "rte_lpm_neon.h"
#elif defined(RTE_ARCH_PPC_64)
#include "rte_lpm_altivec.h"
#else
#include "rte_lpm_sse.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* _RTE_LPM_H_ */
