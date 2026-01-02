#ifndef NRF_H
#define NRF_H

#include "main.h"
#include <stdint.h>
#include <string.h>

// ============================================================================
// COMMANDES SPI NRF24L01+
// ============================================================================
#define NRF24_R_REGISTER        0x00  // Commande lecture registre
#define NRF24_W_REGISTER        0x20  // Commande écriture registre
#define NRF24_R_RX_PAYLOAD      0x61  // Lire payload RX
#define NRF24_W_TX_PAYLOAD      0xA0  // Écrire payload TX
#define NRF24_FLUSH_TX          0xE1  // Vider FIFO TX
#define NRF24_FLUSH_RX          0xE2  // Vider FIFO RX
#define NRF24_REUSE_TX_PL       0xE3  // Réutiliser dernier payload TX
#define NRF24_NOP               0xFF  // No Operation
#define PAYLOAD_SIZE    32

// ============================================================================
// REGISTRES NRF24L01+
// ============================================================================
#define NRF24_CONFIG            0x00  // Configuration
#define NRF24_EN_AA             0x01  // Enable Auto Acknowledgement
#define NRF24_EN_RXADDR         0x02  // Enabled RX Addresses
#define NRF24_SETUP_AW          0x03  // Setup Address Width
#define NRF24_SETUP_RETR        0x04  // Setup Retransmission
#define NRF24_RF_CH             0x05  // RF Channel
#define NRF24_RF_SETUP          0x06  // RF Setup
#define NRF24_STATUS            0x07  // Status
#define NRF24_OBSERVE_TX        0x08  // Transmit observe
#define NRF24_RPD               0x09  // Received Power Detector
#define NRF24_RX_ADDR_P0        0x0A  // RX Address Pipe 0
#define NRF24_RX_ADDR_P1        0x0B  // RX Address Pipe 1
#define NRF24_RX_ADDR_P2        0x0C  // RX Address Pipe 2
#define NRF24_RX_ADDR_P3        0x0D  // RX Address Pipe 3
#define NRF24_RX_ADDR_P4        0x0E  // RX Address Pipe 4
#define NRF24_RX_ADDR_P5        0x0F  // RX Address Pipe 5
#define NRF24_TX_ADDR           0x10  // TX Address
#define NRF24_RX_PW_P0          0x11  // RX Payload Width Pipe 0
#define NRF24_RX_PW_P1          0x12  // RX Payload Width Pipe 1
#define NRF24_RX_PW_P2          0x13  // RX Payload Width Pipe 2
#define NRF24_RX_PW_P3          0x14  // RX Payload Width Pipe 3
#define NRF24_RX_PW_P4          0x15  // RX Payload Width Pipe 4
#define NRF24_RX_PW_P5          0x16  // RX Payload Width Pipe 5
#define NRF24_FIFO_STATUS       0x17  // FIFO Status
#define NRF24_DYNPD             0x1C  // Enable dynamic payload length
#define NRF24_FEATURE           0x1D  // Feature register

// ============================================================================
// BITS DU REGISTRE CONFIG (0x00)
// ============================================================================
#define NRF24_CONFIG_MASK_RX_DR     6  // Mask interrupt RX_DR
#define NRF24_CONFIG_MASK_TX_DS     5  // Mask interrupt TX_DS
#define NRF24_CONFIG_MASK_MAX_RT    4  // Mask interrupt MAX_RT
#define NRF24_CONFIG_EN_CRC         3  // Enable CRC
#define NRF24_CONFIG_CRCO           2  // CRC encoding (0=1byte, 1=2bytes)
#define NRF24_CONFIG_PWR_UP         1  // Power up
#define NRF24_CONFIG_PRIM_RX        0  // RX/TX control (1=RX, 0=TX)

// ============================================================================
// BITS DU REGISTRE STATUS (0x07)
// ============================================================================
#define NRF24_STATUS_RX_DR          6  // Data Ready RX FIFO
#define NRF24_STATUS_TX_DS          5  // Data Sent TX FIFO
#define NRF24_STATUS_MAX_RT         4  // Maximum retransmissions
#define NRF24_STATUS_RX_P_NO        1  // RX pipe number (bits 3:1)
#define NRF24_STATUS_TX_FULL        0  // TX FIFO full

// ============================================================================
// BITS DU REGISTRE FIFO_STATUS (0x17)
// ============================================================================
#define NRF24_FIFO_TX_REUSE         6  // Reuse last TX payload
#define NRF24_FIFO_TX_FULL          5  // TX FIFO full
#define NRF24_FIFO_TX_EMPTY         4  // TX FIFO empty
#define NRF24_FIFO_RX_FULL          1  // RX FIFO full
#define NRF24_FIFO_RX_EMPTY         0  // RX FIFO empty

// ============================================================================
// STRUCTURE DES REGISTRES
// ============================================================================
typedef struct {
    uint8_t CONFIG;
    uint8_t EN_AA;
    uint8_t EN_RXADDR;
    uint8_t RF_CH;
    uint8_t RF_SETUP;
} NRF24_Registers;

enum {
	LISTENER = 0,
	SENDER = 1
};

typedef uint8_t STATE_NRF;
extern uint8_t payload[PAYLOAD_SIZE];
// ============================================================================
// VARIABLES EXTERNES
// ============================================================================
extern SPI_HandleTypeDef hspi1;
extern uint8_t nrf24_tx_address[5];
extern uint8_t nrf24_rx_address[5];
// ============================================================================
// FONCTIONS PUBLIQUES - INITIALISATION
// ============================================================================

/**
 * @brief   Initialisation complète du module NRF24L01+
 * @param   channel : Canal RF (0-125)
 * @param   data_rate : Vitesse (0=1Mbps, 1=2Mbps, 2=250kbps)
 * @param   power : Puissance TX (0=-18dBm, 1=-12dBm, 2=-6dBm, 3=0dBm)
 * @return  0 si succès, -1 si échec
 */
int8_t nrf24_init(uint8_t channel, uint8_t data_rate, uint8_t power);

/**
 * @brief   Configure l'adresse de réception
 * @param   addr : Adresse 5 octets
 * @param   pipe : Numéro du pipe (0 ou 1)
 */
void nrf24_set_rx_address(uint8_t *addr, uint8_t pipe);

/**
 * @brief   Configure l'adresse de transmission
 * @param   addr : Adresse 5 octets
 */
void nrf24_set_tx_address(uint8_t *addr);

// ============================================================================
// FONCTIONS PUBLIQUES - MODE RX
// ============================================================================

/**
 * @brief   Bascule le module en mode réception
 */
void nrf24_start_listening(void);

/**
 * @brief   Arrête le mode réception
 */
void nrf24_stop_listening(void);

/**
 * @brief   Vérifie si des données sont disponibles
 * @return  1 si données disponibles, 0 sinon
 */
uint8_t nrf24_available(void);

/**
 * @brief   Lit un paquet reçu
 * @param   buf : Buffer de destination
 * @param   len : Nombre d'octets à lire
 * @return  1 si succès, 0 si aucune donnée
 */
uint8_t nrf24_read(uint8_t *buf, uint8_t len);

// ============================================================================
// FONCTIONS PUBLIQUES - MODE TX
// ============================================================================

/**
 * @brief   Envoie un paquet de données
 * @param   data : Données à envoyer
 * @param   len : Taille des données (max 32 octets)
 * @return  1 si succès (ACK reçu), 0 si échec
 */
uint8_t nrf24_write(uint8_t *data, uint8_t len);

// ============================================================================
// FONCTIONS PUBLIQUES - GESTION POWER
// ============================================================================

/**
 * @brief   Met le module en power down (économie d'énergie)
 */
void nrf24_power_down(void);

/**
 * @brief   Réveille le module (power up)
 */
void nrf24_power_up(void);

// ============================================================================
// FONCTIONS PUBLIQUES - DEBUG
// ============================================================================

/**
 * @brief   Affiche l'état complet du module (debug)
 * @param   print_func : Fonction printf (ex: printf)
 */
void nrf24_print_status(void (*print_func)(const char*, ...));
void nrf24_read_register(uint8_t reg, uint8_t *data, uint8_t len);
void check_config(uint8_t config, uint8_t status, uint8_t fifo, uint8_t en_aa, uint8_t en_rxaddr,uint8_t rx_addr_p0[5]);
void switchState(STATE_NRF state);

/**
 * @brief Envoie un paquet et attend une réponse avec basculement auto
 */
uint8_t nrf24_write_and_wait_response(uint8_t *tx_data, uint8_t tx_len,
                                       uint8_t *rx_buffer, uint8_t rx_len,
                                       uint32_t timeout_ms);
#endif /* NRF_H */
