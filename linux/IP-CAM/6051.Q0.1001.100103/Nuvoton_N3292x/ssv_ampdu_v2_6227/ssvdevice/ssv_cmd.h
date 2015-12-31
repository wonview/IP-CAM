#ifndef _SSV_CMD_H_
#define _SSV_CMD_H_


#define CLI_BUFFER_SIZE             256
#define CLI_ARG_SIZE                10

#define CLI_RESULT_BUF_SIZE         (4096)

struct ssv_cmd_table {
    const char *cmd;
    int (*cmd_func_ptr)(int, char **);
    const char *usage;
};



#define SSV_REG_READ1(ops, reg, val) \
        (ops)->ifops->readreg((ops)->dev, reg, val)
#define SSV_REG_WRITE1(ops, reg, val) \
        (ops)->ifops->writereg((ops)->dev, reg, val)
#define SSV_REG_SET_BITS1(ops, reg, set, clr) \
    {                                           \
        u32 reg_val;                            \
        SSV_REG_READ(ops, reg, &reg_val);        \
        reg_val &= ~(clr);                      \
        reg_val |= (set);                       \
        SSV_REG_WRITE(ops, reg, reg_val);        \
    }


int ssv_cmd_submit(char *cmd);



#endif /* _SSV_CMD_H_ */


