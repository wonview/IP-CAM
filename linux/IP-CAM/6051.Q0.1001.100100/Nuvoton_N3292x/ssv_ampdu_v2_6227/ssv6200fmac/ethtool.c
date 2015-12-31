#include <linux/hardirq.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/delay.h>

#include "decl.h"
#include "cmd.h"
//SSV -- #include "mesh.h"


static void cabrio_ethtool_get_drvinfo(struct net_device *dev,
					 struct ethtool_drvinfo *info)
{
#if 1
	snprintf(info->fw_version, 32, "0.1");
	strcpy(info->driver, "cabrio");
	strcpy(info->version, cabrio_driver_version);
#else
	struct cabrio_private *priv = dev->ml_priv;

	snprintf(info->fw_version, 32, "%u.%u.%u.p%u",
		priv->fwrelease >> 24 & 0xff,
		priv->fwrelease >> 16 & 0xff,
		priv->fwrelease >>  8 & 0xff,
		priv->fwrelease       & 0xff);
	strcpy(info->driver, "cabrio");
	strcpy(info->version, cabrio_driver_version);
#endif
	}

/*
 * All 8388 parts have 16KiB EEPROM size at the time of writing.
 * In case that changes this needs fixing.
 */
#define CABRIO_EEPROM_LEN 16384

static int cabrio_ethtool_get_eeprom_len(struct net_device *dev)
{
#if 1
    CABRIO_TODO(__func__);
    return 0;
#else
	return CABRIO_EEPROM_LEN;
#endif
}

static int cabrio_ethtool_get_eeprom(struct net_device *dev,
                                  struct ethtool_eeprom *eeprom, u8 * bytes)
{
#if 1
    CABRIO_TODO(__func__);
    return 0;
#else
	struct cabrio_private *priv = dev->ml_priv;
	struct cmd_ds_802_11_eeprom_access cmd;
	int ret;

	cabrio_dbg_enter(CABRIO_DBG_ETHTOOL);

	if (eeprom->offset + eeprom->len > CABRIO_EEPROM_LEN ||
	    eeprom->len > CABRIO_EEPROM_READ_LEN) {
		ret = -EINVAL;
		goto out;
	}

	cmd.hdr.size = cpu_to_le16(sizeof(struct cmd_ds_802_11_eeprom_access) -
		CABRIO_EEPROM_READ_LEN + eeprom->len);
	cmd.action = cpu_to_le16(CMD_ACT_GET);
	cmd.offset = cpu_to_le16(eeprom->offset);
	cmd.len    = cpu_to_le16(eeprom->len);
	ret = cabrio_cmd_with_response(priv, CMD_802_11_EEPROM_ACCESS, &cmd);
	if (!ret)
		memcpy(bytes, cmd.value, eeprom->len);

out:
	cabrio_dbg_leave_args(CABRIO_DBG_ETHTOOL, "ret %d", ret);
        return ret;
#endif
}

static void cabrio_ethtool_get_wol(struct net_device *dev,
				struct ethtool_wolinfo *wol)
{
#if 1
    CABRIO_TODO(__func__);
#else
	struct cabrio_private *priv = dev->ml_priv;

	wol->supported = WAKE_UCAST|WAKE_MCAST|WAKE_BCAST|WAKE_PHY;

	if (priv->wol_criteria == EHS_REMOVE_WAKEUP)
		return;

	if (priv->wol_criteria & EHS_WAKE_ON_UNICAST_DATA)
		wol->wolopts |= WAKE_UCAST;
	if (priv->wol_criteria & EHS_WAKE_ON_MULTICAST_DATA)
		wol->wolopts |= WAKE_MCAST;
	if (priv->wol_criteria & EHS_WAKE_ON_BROADCAST_DATA)
		wol->wolopts |= WAKE_BCAST;
	if (priv->wol_criteria & EHS_WAKE_ON_MAC_EVENT)
		wol->wolopts |= WAKE_PHY;
#endif // 0
}

static int cabrio_ethtool_set_wol(struct net_device *dev,
			       struct ethtool_wolinfo *wol)
{
#if 1
    CABRIO_TODO(__func__);
#else
	struct cabrio_private *priv = dev->ml_priv;

	if (wol->wolopts & ~(WAKE_UCAST|WAKE_MCAST|WAKE_BCAST|WAKE_PHY))
		return -EOPNOTSUPP;

	priv->wol_criteria = 0;
	if (wol->wolopts & WAKE_UCAST)
		priv->wol_criteria |= EHS_WAKE_ON_UNICAST_DATA;
	if (wol->wolopts & WAKE_MCAST)
		priv->wol_criteria |= EHS_WAKE_ON_MULTICAST_DATA;
	if (wol->wolopts & WAKE_BCAST)
		priv->wol_criteria |= EHS_WAKE_ON_BROADCAST_DATA;
	if (wol->wolopts & WAKE_PHY)
		priv->wol_criteria |= EHS_WAKE_ON_MAC_EVENT;
	if (wol->wolopts == 0)
		priv->wol_criteria |= EHS_REMOVE_WAKEUP;
#endif
	return 0;
}

const struct ethtool_ops cabrio_ethtool_ops = {
	.get_drvinfo = cabrio_ethtool_get_drvinfo,
	.get_eeprom =  cabrio_ethtool_get_eeprom,
	.get_eeprom_len = cabrio_ethtool_get_eeprom_len,
//SSV -- 
//#ifdef CONFIG_LIBERTAS_MESH
#if 0
	.get_sset_count = cabrio_mesh_ethtool_get_sset_count,
	.get_ethtool_stats = cabrio_mesh_ethtool_get_stats,
	.get_strings = cabrio_mesh_ethtool_get_strings,
#endif
	.get_wol = cabrio_ethtool_get_wol,
	.set_wol = cabrio_ethtool_set_wol,
};

