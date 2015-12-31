#ifndef _CABRIO_DEBUGFS_H_
#define _CABRIO_DEBUGFS_H_

void cabrio_debugfs_init(void);
void cabrio_debugfs_remove(void);

void cabrio_debugfs_init_one(struct cabrio_private *priv, struct net_device *dev);
void cabrio_debugfs_remove_one(struct cabrio_private *priv);

#endif
