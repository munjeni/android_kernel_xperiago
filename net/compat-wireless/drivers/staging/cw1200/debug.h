/*
 * DebugFS code for ST-Ericsson CW1200 mac80211 driver
 *
 * Copyright (c) 2011, ST-Ericsson
 * Copyright (c) 2012, Sony Mobile Communications AB
 *
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CW1200_DEBUG_H_INCLUDED
#define CW1200_DEBUG_H_INCLUDED

#include "itp.h"

struct cw200_common;

#ifdef CONFIG_CW1200_DEBUGFS

#ifdef CONFIG_CW1200_BINARY_LOGGING
#define MAX_BINLOG_BUF_SIZE 4096

struct cw1200_binlog_ctx {
	u8 data[MAX_BINLOG_BUF_SIZE];
	size_t start;
	size_t end;
	u8 text[128]; /* need this one to support printf format */
	spinlock_t lock;
};
#endif /* CONFIG_CW1200_BINARY_LOGGING */

struct cw1200_debug_priv {
	struct dentry *debugfs_phy;
	int tx;
	int tx_agg;
	int rx;
	int rx_agg;
	int tx_multi;
	int tx_multi_frames;
	int tx_cache_miss;
	int tx_align;
	int tx_ttl;
	int tx_burst;
	int rx_burst;
	int ba_cnt;
	int ba_acc;
	int ba_cnt_rx;
	int ba_acc_rx;
#ifdef CONFIG_CW1200_ITP
	struct cw1200_itp itp;
#endif /* CONFIG_CW1200_ITP */
#ifdef CONFIG_CW1200_BINARY_LOGGING
	struct cw1200_binlog_ctx binlog;
#endif /* CONFIG_CW1200_BINARY_LOGGING */
};

int cw1200_debug_init(struct cw1200_common *priv);
void cw1200_debug_release(struct cw1200_common *priv);

static inline void cw1200_debug_txed(struct cw1200_common *priv)
{
	++priv->debug->tx;
}

static inline void cw1200_debug_txed_agg(struct cw1200_common *priv)
{
	++priv->debug->tx_agg;
}

static inline void cw1200_debug_txed_multi(struct cw1200_common *priv,
					   int count)
{
	++priv->debug->tx_multi;
	priv->debug->tx_multi_frames += count;
}

static inline void cw1200_debug_rxed(struct cw1200_common *priv)
{
	++priv->debug->rx;
}

static inline void cw1200_debug_rxed_agg(struct cw1200_common *priv)
{
	++priv->debug->rx_agg;
}

static inline void cw1200_debug_tx_cache_miss(struct cw1200_common *priv)
{
	++priv->debug->tx_cache_miss;
}

static inline void cw1200_debug_tx_align(struct cw1200_common *priv)
{
	++priv->debug->tx_align;
}

static inline void cw1200_debug_tx_ttl(struct cw1200_common *priv)
{
	++priv->debug->tx_ttl;
}

static inline void cw1200_debug_tx_burst(struct cw1200_common *priv)
{
	++priv->debug->tx_burst;
}

static inline void cw1200_debug_rx_burst(struct cw1200_common *priv)
{
	++priv->debug->rx_burst;
}

static inline void cw1200_debug_ba(struct cw1200_common *priv,
				   int ba_cnt, int ba_acc, int ba_cnt_rx,
				   int ba_acc_rx)
{
	priv->debug->ba_cnt = ba_cnt;
	priv->debug->ba_acc = ba_acc;
	priv->debug->ba_cnt_rx = ba_cnt_rx;
	priv->debug->ba_acc_rx = ba_acc_rx;
}

int cw1200_print_fw_version(struct cw1200_common *priv, u8 *buf, size_t len);

typedef enum {
	BINLOG_WSM_PRINTK = 0,
	BINLOG_WSM_TX,
	BINLOG_WSM_RX,
	BINLOG_WSM_EXCEPTION,
} cw1200_binlog_id_t;

#ifdef CONFIG_CW1200_BINARY_LOGGING
void cw1200_binlog_put_data(struct cw1200_common *priv, cw1200_binlog_id_t id,
			    const u8 *data, size_t len);

__attribute__ ((format (printf, 3, 4)))
void cw1200_binlog_put_text(struct cw1200_common *priv, cw1200_binlog_id_t id,
			    const char *fmt, ...);
#else /* CONFIG_CW1200_BINARY_LOGGING */
static inline void cw1200_binlog_put_data(struct cw1200_common *priv, cw1200_binlog_id_t id,
					  const u8 *data, size_t len)
{
}

__attribute__ ((format (printf, 3, 4)))
static inline void cw1200_binlog_put_text(struct cw1200_common *priv, cw1200_binlog_id_t id,
					  const char *fmt, ...)
{
}
#endif /* CONFIG_CW1200_BINARY_LOGGING */
#else /* CONFIG_CW1200_DEBUGFS */

static inline int cw1200_debug_init(struct cw1200_common *priv)
{
	return 0;
}

static inline void cw1200_debug_release(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_txed(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_txed_agg(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_txed_multi(struct cw1200_common *priv,
					   int count)
{
}

static inline void cw1200_debug_rxed(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_rxed_agg(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_tx_cache_miss(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_tx_align(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_tx_ttl(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_tx_burst(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_rx_burst(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_ba(struct cw1200_common *priv,
				   int ba_cnt, int ba_acc, int ba_cnt_rx, int ba_acc_rx)
{
}

int cw1200_print_fw_version(struct cw1200_common *priv, u8 *buf, size_t len)
{
}

#endif /* CONFIG_CW1200_DEBUGFS */

#endif /* CW1200_DEBUG_H_INCLUDED */
