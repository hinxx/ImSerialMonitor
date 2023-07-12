#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>


/*
bits 7 6 5 4 3 2 1 0
     y y y x x x x x

x[5] = param id
x[3] = param data type
 0 0 0 - uint8
 0 0 1 - int8
 0 1 0 - uint16
 0 1 1 - int16
 1 0 0 - uint32
 1 0 1 - int32
#define S16_ID    (3 << 5)
#define S32_ID    (5 << 5)

#define PARAM_A   (0)
#define PARAM_B   (4)
#define PARAM_C   (30)
*/

struct config {
    uint32_t param32;
    uint16_t param16;
    uint8_t param8_1;
    uint8_t param8_2;
};

struct monitor
{
    uint32_t param32_1;
    uint32_t param32_2;
    uint16_t param16_1;
    uint16_t param16_2;
};

enum {
    // 0 .. 127 config parameters
    CFG_X_32,
    CFG_Y_16,
    CFG_ZZ_8_1,
    CFG_WWW_8_2,
    // 128 .. 255 monitor parameters
    MON_A_32 = 128,
    MON_B_32,
    MON_HH_16,
    MON_DD_16,
};

/*
uint32_t pack(uint8_t *buf, uint32_t off, uint32_t type, uint32_t param, void *val) {
    if (type == S16_ID) {
        buf[off++] = (uint8_t)(type | param);
        *(int16_t *)&buf[off] = *(int16_t *)val;
        off += sizeof(int16_t);
    } else if (type == S32_ID) {
        buf[off++] = (uint8_t)(type | param);
        *(int32_t *)&buf[off] = *(int32_t *)val;
        off += sizeof(int32_t);
    }
    return off;
}

uint32_t unpack(const uint8_t *buf, const uint32_t off, uint32_t type, uint32_t param, void *val) {
    uint32_t t = (uint8_t)(buf[off] >> 5);
    uint32_t p = (uint8_t)(buf[off] & 0x1F);
    if (t == type && p == param) {

    }
    return 0;
}
*/

uint16_t _unpack16(uint8_t *buf) {
    return (uint16_t)(buf[1] << 8)
           | (uint8_t)(buf[0]);
}

uint32_t _unpack32(uint8_t *buf) {
    return (uint32_t)(buf[3] << 24)
           | (uint32_t)(buf[2] << 16)
           | (uint32_t)(buf[1] << 8)
           | (uint32_t)(buf[0]);
}

uint8_t _pack16(uint8_t *buf, uint16_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    return 2;
}

uint8_t _pack32(uint8_t *buf, uint32_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
    return 4;
}

// send to remote host, report value status
uint32_t pack1(uint8_t *buf, uint32_t param, struct config *config) {
    uint8_t len = 0;
    if (param == CFG_X_32) {
        buf[len++] = (uint8_t)(CFG_X_32);
        len += _pack32(&buf[len], config->param32);
    } else if (param == CFG_Y_16) {
        buf[len++] = (uint8_t)(CFG_Y_16);
        len += _pack16(&buf[len], config->param16);
    } else if (param == CFG_ZZ_8_1) {
        buf[len++] = (uint8_t)(CFG_ZZ_8_1);
        buf[len++] = (uint8_t)(config->param8_1 & 0xFF);
    } else if (param == CFG_WWW_8_2) {
        buf[len++] = (uint8_t)(CFG_WWW_8_2);
        buf[len++] = (uint8_t)(config->param8_2 & 0xFF);
    }

    return len;
}

uint32_t pack2(uint8_t *buf, uint32_t param, struct monitor *monitor) {
    uint8_t len = 0;
    if (param == MON_A_32) {
        buf[len++] = (uint8_t)(MON_A_32);
        len += _pack32(&buf[len], monitor->param32_1);
    } else if (param == MON_B_32) {
        buf[len++] = (uint8_t)(MON_B_32);
        len += _pack32(&buf[len], monitor->param32_2);
    } else if (param == MON_DD_16) {
        buf[len++] = (uint8_t)(MON_DD_16);
        len += _pack16(&buf[len], monitor->param16_1);
    } else if (param == MON_HH_16) {
        buf[len++] = (uint8_t)(MON_HH_16);
        len += _pack16(&buf[len], monitor->param16_2);
    }

    return len;
}

// receive from remote host, apply value
uint32_t unpack1(uint8_t *buf, struct config *config) {
    if (buf[0] == CFG_X_32) {
        config->param32 = _unpack32(&buf[1]);
    } else if (buf[0] == CFG_Y_16) {
        config->param16 = _unpack16(&buf[1]);
    } else if (buf[0] == CFG_ZZ_8_1) {
        config->param8_1 = buf[1];
    } else if (buf[0] == CFG_WWW_8_2) {
        config->param8_2 = buf[1];
    }

    return 0;
}

void hexdump(const uint8_t *buf, const uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        printf("%02X ", (uint8_t)(buf[i] & 0xFF));
    }
    printf("\n");
}

int main(int argc, char const *argv[]) {
    
    uint8_t buf[100];
    uint32_t len;

    // int16_t param_a = 0x1234;
    // int32_t param_b = 0x6789;
    // len = 0;
    // buf[len++] = (uint8_t)(S16_ID | PARAM_A);
    // buf[len++] = (uint8_t)(param_a & 0xFF);
    // buf[len++] = (uint8_t)((param_a >> 8) & 0xFF);
    // len = pack(buf, len, S16_ID, PARAM_A, &param_a);

    // buf[len++] = (uint8_t)(S32_ID | PARAM_B);
    // buf[len++] = (uint8_t)(param_b & 0xFF);
    // buf[len++] = (uint8_t)((param_b >> 8) & 0xFF);
    // buf[len++] = (uint8_t)((param_b >> 16) & 0xFF);
    // buf[len++] = (uint8_t)((param_b >> 24) & 0xFF);
    // len = pack(buf, len, S32_ID, PARAM_B, &param_b);

    struct config cfg = {0};
    struct config cfg2 = {0};

    cfg.param32 = 0x6789;
    len = pack1(buf, CFG_X_32, &cfg);
    hexdump(buf, len);
    unpack1(buf, &cfg2);
    assert(cfg.param32 == cfg2.param32);
    
    cfg.param16 = 0x1234;
    len = pack1(buf, CFG_Y_16, &cfg);
    hexdump(buf, len);
    unpack1(buf, &cfg2);
    assert(cfg.param16 == cfg2.param16);
    
    cfg.param8_1 = 0xAA;
    len = pack1(buf, CFG_ZZ_8_1, &cfg);
    hexdump(buf, len);
    unpack1(buf, &cfg2);
    assert(cfg.param8_1 == cfg2.param8_1);

    cfg.param8_2 = 0x22;
    len = pack1(buf, CFG_WWW_8_2, &cfg);
    hexdump(buf, len);
    unpack1(buf, &cfg2);
    assert(cfg.param8_2 == cfg2.param8_2);

    memset(buf, 0, sizeof(buf));
    len = 0;
    struct monitor mon = {0};
    mon.param32_1 = 0x10102020;
    mon.param32_2 = 0x56561212;
    mon.param16_1 = 0x6677;
    mon.param16_2 = 0x1337;
    len += pack2(&buf[len], MON_A_32, &mon);
    len += pack2(&buf[len], MON_B_32, &mon);
    len += pack2(&buf[len], MON_DD_16, &mon);
    len += pack2(&buf[len], MON_HH_16, &mon);
    hexdump(buf, len);

    return 0;
}
