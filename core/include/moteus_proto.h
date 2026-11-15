/*
 * moteus_proto.h - codificação das tramas CAN-FD do controlador mjbots moteus r4.11.
 * Formato conforme a documentação oficial (protocol/can.md, protocol/registers.md).
 * Puro (sem I/O): o envio pelo MCP2518FD é feito na camada de firmware.
 */
#ifndef MOTEUS_PROTO_H
#define MOTEUS_PROTO_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  have_mode, have_motion, have_aux;
    uint8_t  mode, fault;
    float    position_rev, velocity_rev_s, torque_nm;
    float    voltage_v, temp_c;
} moteus_status_t;

/* ID CAN: bits 15..8 = origem (bit 15 pede resposta), 7..0 = destino */
uint32_t moteus_can_id(uint8_t src, uint8_t dst, int want_reply);

/* Modo posição com posição=NaN (= controlo de velocidade), + pedido de estado. Devolve nº de bytes
 * (já preenchido com NOP até um comprimento CAN-FD válido). buf >= 64 bytes. */
/* Invalid numeric arguments produce a STOP frame (same query setting). */
size_t moteus_pack_velocity(uint8_t *buf, float vel_rev_s, float max_torque_nm, float accel_rev_s2, int query);
size_t moteus_pack_stop(uint8_t *buf, int query);

/* On error, *st is zero. Group flags require all 3 registers in that group. */
int moteus_parse_reply(const uint8_t *buf, size_t n, moteus_status_t *st);

#ifdef __cplusplus
}
#endif
#endif
