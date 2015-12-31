/* Copyright (C) 2007, Red Hat, Inc. */

#ifndef _CABRIO_CMD_H_
#define _CABRIO_CMD_H_

#include <net/cfg80211.h>

#include "host.h"
#include "dev.h"
#include "cabrio-if/cmd_def.h"


/* Command & response transfer between host and card */

struct host_cmd_node {
	struct list_head list;
	int result;
	/* command response */
	int (*callback)(struct cabrio_private *,
        			unsigned long,
			        HDR_HostEvent *);
	unsigned long callback_arg;
	/* command data */
	//struct cmd_header *cmdbuf;
	HDR_HostCmd     *cmdbuf;
	/* wait queue */
	u16 cmdwaitqwoken;
	wait_queue_head_t cmdwait_q;
};


/* cabrio_cmd() infers the size of the buffer to copy data back into, from
   the size of the target of the pointer. Since the command to be sent
   may often be smaller, that size is set in cmd->size by the caller.*/
#if 0
#define cabrio_cmd(priv, cmdnr, cmd, cb, cb_arg)	({		\
	uint16_t __sz = le16_to_cpu((cmd)->len);		\
	(cmd)->hdr.size = cpu_to_le16(sizeof(*(cmd)));		\
	__cabrio_cmd(priv, cmdnr, &(cmd)->hdr, __sz, cb, cb_arg);	\
})
#endif 
#define cabrio_cmd(priv, cmdnr, cmd, cb, cb_arg)	({		\
	uint16_t __sz = (cmd)->len;		\
	(cmd)->h_cmd = cmdnr; \
	__cabrio_cmd(priv, cmdnr, (const u8 *)cmd, __sz, cb, cb_arg);	\
})

#define cabrio_cmd_with_response(priv, cmdnr, cmd, event)	\
	cabrio_cmd(priv, cmdnr, cmd, cabrio_cmd_copyback, (unsigned long)(event))

void cabrio_cmd_async(struct cabrio_private *priv, uint16_t command,
                      HDR_HostCmd *in_cmd, int in_cmd_size);


int __cabrio_cmd(struct cabrio_private *priv, uint16_t command,
	             const u8 *in_cmd, int in_cmd_size,
	             int (*callback)(struct cabrio_private *, unsigned long, HDR_HostEvent *),
	             unsigned long callback_arg);


struct host_cmd_node *__cabrio_cmd_async(struct cabrio_private *priv,
                                         uint16_t command, const u8 *in_cmd, int in_cmd_size,
                                         int (*callback)(struct cabrio_private *, unsigned long, HDR_HostEvent *),
                                         unsigned long callback_arg);

int cabrio_cmd_copyback(struct cabrio_private *priv, unsigned long extra,
		                HDR_HostEvent *resp);

int cabrio_allocate_cmd_buffer(struct cabrio_private *priv);
int cabrio_free_cmd_buffer(struct cabrio_private *priv);

int cabrio_execute_next_command(struct cabrio_private *priv);
void __cabrio_complete_command(struct cabrio_private *priv, struct host_cmd_node *cmd,
                               int result);
void cabrio_complete_command(struct cabrio_private *priv, struct host_cmd_node *cmd,
                             int result);
int cabrio_process_command_response(struct cabrio_private *priv, u8 *data, u32 len);


/* From cmdresp.c */

void cabrio_mac_event_disconnected(struct cabrio_private *priv);

/* Events */

int cabrio_process_event(struct cabrio_private *priv, void *event);

/* Actual commands */

int cabrio_update_hw_spec(struct cabrio_private *priv);

int cabrio_set_channel(struct cabrio_private *priv, u8 channel);

int cabrio_update_channel(struct cabrio_private *priv);

int cabrio_host_sleep_cfg(struct cabrio_private *priv, uint32_t criteria,
                          struct wol_config *p_wol_config);

int cabrio_cmd_802_11_sleep_params(struct cabrio_private *priv, uint16_t cmd_action,
                                   struct sleep_params *sp);

void cabrio_ps_confirm_sleep(struct cabrio_private *priv);

int cabrio_set_radio(struct cabrio_private *priv, u8 preamble, u8 radio_on);

void cabrio_set_mac_control(struct cabrio_private *priv);

int cabrio_get_tx_power(struct cabrio_private *priv, s16 *curlevel, s16 *minlevel,
                        s16 *maxlevel);

#if ORIG
int cabrio_set_snmp_mib(struct cabrio_private *priv, u32 oid, u16 val);

int cabrio_get_snmp_mib(struct cabrio_private *priv, u32 oid, u16 *out_val);
#endif // ORIG

/* Commands only used in wext.c, assoc. and scan.c */

int cabrio_set_power_adapt_cfg(struct cabrio_private *priv, int enable, int8_t p0,
                               int8_t p1, int8_t p2);

int cabrio_set_tpc_cfg(struct cabrio_private *priv, int enable, int8_t p0, int8_t p1,
                       int8_t p2, int usesnr);

int cabrio_set_data_rate(struct cabrio_private *priv, u8 rate);

int cabrio_cmd_802_11_rate_adapt_rateset(struct cabrio_private *priv,
                                         uint16_t cmd_action);

int cabrio_set_tx_power(struct cabrio_private *priv, s16 dbm);

int cabrio_set_deep_sleep(struct cabrio_private *priv, int deep_sleep);

int cabrio_set_host_sleep(struct cabrio_private *priv, int host_sleep);

int cabrio_set_monitor_mode(struct cabrio_private *priv, int enable);

int cabrio_get_rssi(struct cabrio_private *priv, s8 *snr, s8 *nf);

int cabrio_set_11d_domain_info(struct cabrio_private *priv,
                               struct regulatory_request *request,
                               struct ieee80211_supported_band **bands);

int cabrio_get_reg(struct cabrio_private *priv, u16 reg, u16 offset, u32 *value);

int cabrio_set_reg(struct cabrio_private *priv, u16 reg, u16 offset, u32 value);

int cabrio_set_ps_mode(struct cabrio_private *priv, u16 cmd_action, bool block);

#endif /* _CABRIO_CMD_H */
