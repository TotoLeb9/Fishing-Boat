#include "nrf.h"

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================
static NRF24_Registers nrf24_regs;
static uint8_t nrf24_tx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
static uint8_t nrf24_rx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

// ============================================================================
// FONCTIONS PRIVÉES - CONTRÔLE GPIO
// ============================================================================

/**
 * @brief  Met le pin CSN à HIGH (fin communication SPI)
 */
static inline void cs_high(void) {
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Met le pin CSN à LOW (début communication SPI)
 */
static inline void cs_low(void) {
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Met le pin CE à HIGH (mode actif TX/RX)
 */
static inline void ce_high(void) {
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Met le pin CE à LOW (mode standby)
 */
static inline void ce_low(void) {
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}

// ============================================================================
// FONCTIONS PRIVÉES - SPI & REGISTRES
// ============================================================================

/**
 * @brief   Transfert SPI full-duplex
 * @param   data : Octet à envoyer
 * @return  Octet reçu
 */
static uint8_t spi_transfer(uint8_t data) {
    uint8_t received = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &received, 1, HAL_MAX_DELAY);
    return received;
}

/**
 * @brief   Écriture dans un registre NRF24
 * @param   reg : Adresse du registre (0x00 à 0x1F)
 * @param   data : Données à écrire
 * @param   len : Nombre d'octets
 */
void nrf24_write_register(uint8_t reg, uint8_t *data, uint8_t len) {
    cs_low();
    spi_transfer(0x20 | (reg & 0x1F));  // Commande W_REGISTER
    for (uint8_t i = 0; i < len; i++) {
        spi_transfer(data[i]);
    }
    cs_high();
}

/**
 * @brief   Lecture d'un registre NRF24
 * @param   reg : Adresse du registre (0x00 à 0x1F)
 * @param   data : Buffer de destination
 * @param   len : Nombre d'octets à lire
 */
void nrf24_read_register(uint8_t reg, uint8_t *data, uint8_t len) {
    cs_low();
    spi_transfer(0x00 | (reg & 0x1F));  // Commande R_REGISTER
    for (uint8_t i = 0; i < len; i++) {
        data[i] = spi_transfer(0xFF);
    }
    cs_high();
}

/**
 * @brief   Envoie une commande simple (sans paramètre)
 * @param   cmd : Commande (ex: FLUSH_TX, FLUSH_RX)
 * @return  Registre STATUS
 */
static uint8_t nrf24_send_command(uint8_t cmd) {
    uint8_t status;
    cs_low();
    status = spi_transfer(cmd);
    cs_high();
    return status;
}

/**
 * @brief   Vide le buffer TX FIFO
 */
static void flush_tx(void) {
    nrf24_send_command(0xE1);  // FLUSH_TX
}

/**
 * @brief   Vide le buffer RX FIFO
 */
static void flush_rx(void) {
    nrf24_send_command(0xE2);  // FLUSH_RX
}

/**
 * @brief   Lit le registre STATUS
 * @return  Valeur du registre STATUS
 */
static uint8_t read_status(void) {
    uint8_t status;
    cs_low();
    status = spi_transfer(0xFF);  // NOP command
    cs_high();
    return status;
}

/**
 * @brief   Efface les flags d'interruption dans STATUS
 * @param   flags : Masque des bits à effacer (RX_DR | TX_DS | MAX_RT)
 */
static void clear_status_flags(uint8_t flags) {
    nrf24_write_register(NRF24_STATUS, &flags, 1);
}

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
int8_t nrf24_init(uint8_t channel, uint8_t data_rate, uint8_t power) {
    // Sécurité : CE et CSN à l'état initial
    ce_low();
    cs_high();
    HAL_Delay(5);  // Attendre stabilisation alimentation (4ms min datasheet)

    // ========== CONFIG ==========
    // PWR_UP=0, PRIM_RX=0, CRC 2 bytes, interruptions activées
    nrf24_regs.CONFIG = 0x08;  // 0b00001000
    nrf24_write_register(NRF24_CONFIG, &nrf24_regs.CONFIG, 1);
    HAL_Delay(2);

    // ========== EN_AA (Auto Acknowledgement) ==========
    // Activer ACK sur tous les pipes
    nrf24_regs.EN_AA = 0x3F;  // 0b00111111
    nrf24_write_register(NRF24_EN_AA, &nrf24_regs.EN_AA, 1);

    // ========== EN_RXADDR (Enable RX Addresses) ==========
    // Activer pipe 0 et pipe 1
    nrf24_regs.EN_RXADDR = 0x03;  // 0b00000011
    nrf24_write_register(NRF24_EN_RXADDR, &nrf24_regs.EN_RXADDR, 1);

    // ========== SETUP_AW (Address Width) ==========
    // Adresses de 5 octets
    uint8_t addr_width = 0x03;
    nrf24_write_register(NRF24_SETUP_AW, &addr_width, 1);

    // ========== SETUP_RETR (Auto Retransmission) ==========
    // ARD=500µs (0x1), ARC=15 retries (0xF)
    uint8_t setup_retr = 0x1F;  // 0b00011111
    nrf24_write_register(NRF24_SETUP_RETR, &setup_retr, 1);

    // ========== RF_CH (RF Channel) ==========
    if (channel > 125) channel = 76;  // Canal par défaut
    nrf24_write_register(NRF24_RF_CH, &channel, 1);

    // ========== RF_SETUP (RF Setup) ==========
    // Configuration vitesse + puissance
    nrf24_regs.RF_SETUP = 0x00;

    // Vitesse de données
    switch(data_rate) {
        case 0:  // 1 Mbps
            nrf24_regs.RF_SETUP &= ~((1<<5) | (1<<3));
            break;
        case 1:  // 2 Mbps
            nrf24_regs.RF_SETUP &= ~(1<<5);
            nrf24_regs.RF_SETUP |= (1<<3);
            break;
        case 2:  // 250 kbps
            nrf24_regs.RF_SETUP |= (1<<5);
            nrf24_regs.RF_SETUP &= ~(1<<3);
            break;
        default:  // 1 Mbps par défaut
            nrf24_regs.RF_SETUP &= ~((1<<5) | (1<<3));
    }

    // Puissance TX
    nrf24_regs.RF_SETUP |= ((power & 0x03) << 1);
    nrf24_write_register(NRF24_RF_SETUP, &nrf24_regs.RF_SETUP, 1);

    // ========== ADRESSES RX ==========
    // Pipe 0 et Pipe 1 avec adresses par défaut
    nrf24_write_register(NRF24_RX_ADDR_P0, nrf24_rx_address, 5);
    nrf24_write_register(NRF24_RX_ADDR_P1, nrf24_rx_address, 5);

    // Adresse TX (nécessaire pour ACK en mode RX)
    nrf24_write_register(NRF24_TX_ADDR, nrf24_tx_address, 5);

    // ========== RX_PW (Payload Width) ==========
    // Taille payload 32 octets sur pipe 0 et 1
    uint8_t payload_size = 32;
    nrf24_write_register(NRF24_RX_PW_P0, &payload_size, 1);
    nrf24_write_register(NRF24_RX_PW_P1, &payload_size, 1);

    // ========== DYNPD & FEATURE (désactivés) ==========
    uint8_t zero = 0x00;
    nrf24_write_register(NRF24_DYNPD, &zero, 1);
    nrf24_write_register(NRF24_FEATURE, &zero, 1);

    // Vider les FIFOs
    flush_tx();
    flush_rx();

    // Effacer les flags
    clear_status_flags(0x70);  // RX_DR | TX_DS | MAX_RT

    // Power up en mode standby
    nrf24_regs.CONFIG |= (1<<1);  // PWR_UP
    nrf24_write_register(NRF24_CONFIG, &nrf24_regs.CONFIG, 1);
    HAL_Delay(2);  // Attendre 1.5ms pour power up

    return 0;
}

/**
 * @brief   Configure l'adresse de réception
 * @param   addr : Adresse 5 octets
 * @param   pipe : Numéro du pipe (0 ou 1)
 */
void nrf24_set_rx_address(uint8_t *addr, uint8_t pipe) {
    if (pipe == 0) {
        memcpy(nrf24_rx_address, addr, 5);
        nrf24_write_register(NRF24_RX_ADDR_P0, addr, 5);
    } else if (pipe == 1) {
        nrf24_write_register(NRF24_RX_ADDR_P1, addr, 5);
    }
}

/**
 * @brief   Configure l'adresse de transmission
 * @param   addr : Adresse 5 octets
 */
void nrf24_set_tx_address(uint8_t *addr) {
    memcpy(nrf24_tx_address, addr, 5);
    nrf24_write_register(NRF24_TX_ADDR, addr, 5);
    // En mode TX avec ACK, pipe 0 doit avoir la même adresse
    nrf24_write_register(NRF24_RX_ADDR_P0, addr, 5);
}

// ============================================================================
// FONCTIONS PUBLIQUES - MODE RX
// ============================================================================

/**
 * @brief   Bascule le module en mode réception
 */
void nrf24_start_listening(void) {
    // Lire CONFIG actuel
    nrf24_read_register(NRF24_CONFIG, &nrf24_regs.CONFIG, 1);

    // Passer en mode RX
    nrf24_regs.CONFIG |= (1<<0);   // PRIM_RX = 1
    nrf24_regs.CONFIG |= (1<<1);   // PWR_UP = 1
    nrf24_write_register(NRF24_CONFIG, &nrf24_regs.CONFIG, 1);

    // Restaurer l'adresse RX pipe 0
    nrf24_write_register(NRF24_RX_ADDR_P0, nrf24_rx_address, 5);

    // Effacer les flags
    clear_status_flags(0x70);

    // Vider RX FIFO
    flush_rx();

    // Activer RX
    ce_high();
    HAL_Delay(1);  // Délai stabilisation (130µs min)
}

/**
 * @brief   Arrête le mode réception
 */
void nrf24_stop_listening(void) {
    ce_low();
    HAL_Delay(1);

    // Vider les FIFOs
    flush_tx();
    flush_rx();
}

/**
 * @brief   Vérifie si des données sont disponibles
 * @return  1 si données disponibles, 0 sinon
 */
uint8_t nrf24_available(void) {
    uint8_t status = read_status();

    // Bit 6 (RX_DR) = 1 si données reçues
    if (status & (1<<6)) {
        return 1;
    }

    // Alternative : vérifier FIFO_STATUS
    uint8_t fifo_status;
    nrf24_read_register(NRF24_FIFO_STATUS, &fifo_status, 1);
    return !(fifo_status & 0x01);  // RX_EMPTY = 0 si FIFO non vide
}

/**
 * @brief   Lit un paquet reçu
 * @param   buf : Buffer de destination
 * @param   len : Nombre d'octets à lire
 * @return  1 si succès, 0 si aucune donnée
 */
uint8_t nrf24_read(uint8_t *buf, uint8_t len) {
    if (!nrf24_available()) {
        return 0;
    }

    cs_low();
    spi_transfer(NRF24_R_RX_PAYLOAD);  // 0x61
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = spi_transfer(0xFF);
    }
    cs_high();

    // Effacer flag RX_DR
    clear_status_flags(1<<6);

    return 1;
}

// ============================================================================
// FONCTIONS PUBLIQUES - MODE TX
// ============================================================================

/**
 * @brief   Envoie un paquet de données
 * @param   data : Données à envoyer
 * @param   len : Taille des données (max 32 octets)
 * @return  1 si succès (ACK reçu), 0 si échec
 */
uint8_t nrf24_write(uint8_t *data, uint8_t len) {
    // S'assurer qu'on est en mode TX
    ce_low();

    // Lire CONFIG actuel
    nrf24_read_register(NRF24_CONFIG, &nrf24_regs.CONFIG, 1);

    // Configurer en mode TX
    nrf24_regs.CONFIG &= ~(1<<0);  // PRIM_RX = 0
    nrf24_regs.CONFIG |= (1<<1);   // PWR_UP = 1
    nrf24_write_register(NRF24_CONFIG, &nrf24_regs.CONFIG, 1);
    HAL_Delay(1);  // Attendre transition RX→TX

    // Vérifier que l'adresse TX pipe 0 est correcte (pour ACK)
    nrf24_write_register(NRF24_RX_ADDR_P0, nrf24_tx_address, 5);

    // Effacer les anciens flags
    clear_status_flags(0x70);

    // Écrire le payload
    cs_low();
    spi_transfer(NRF24_W_TX_PAYLOAD);  // 0xA0
    for (uint8_t i = 0; i < len; i++) {
        spi_transfer(data[i]);
    }
    cs_high();

    // Pulse CE pour démarrer la transmission (min 10µs)
    ce_high();
    HAL_Delay(1);  // 1ms = largement suffisant
    ce_low();

    // Attendre TX_DS (succès) ou MAX_RT (échec)
    uint8_t status;
    uint32_t timeout = HAL_GetTick() + 100;  // Timeout 100ms

    while (1) {
        status = read_status();

        // TX_DS (bit 5) = transmission réussie
        if (status & (1<<5)) {
            clear_status_flags(1<<5);
            flush_tx();
            return 1;
        }

        // MAX_RT (bit 4) = échec après max retries
        if (status & (1<<4)) {
            clear_status_flags(1<<4);
            flush_tx();
            return 0;
        }

        // Timeout
        if (HAL_GetTick() > timeout) {
            flush_tx();
            return 0;
        }
    }
}

/**
 * @brief   Met le module en power down (économie d'énergie)
 */
void nrf24_power_down(void) {
    ce_low();
    nrf24_read_register(NRF24_CONFIG, &nrf24_regs.CONFIG, 1);
    nrf24_regs.CONFIG &= ~(1<<1);  // PWR_UP = 0
    nrf24_write_register(NRF24_CONFIG, &nrf24_regs.CONFIG, 1);
}

/**
 * @brief   Réveille le module (power up)
 */
void nrf24_power_up(void) {
    nrf24_read_register(NRF24_CONFIG, &nrf24_regs.CONFIG, 1);
    nrf24_regs.CONFIG |= (1<<1);  // PWR_UP = 1
    nrf24_write_register(NRF24_CONFIG, &nrf24_regs.CONFIG, 1);
    HAL_Delay(2);  // Attendre 1.5ms
}

// ============================================================================
// FONCTIONS PUBLIQUES - DEBUG
// ============================================================================

/**
 * @brief   Affiche l'état complet du module (debug)
 * @param   print_func : Fonction printf (ex: printf ou HAL_UART_Transmit wrapper)
 */
void nrf24_print_status(void (*print_func)(const char*, ...)) {
    uint8_t config, status, rf_ch, rf_setup, fifo_status;

    nrf24_read_register(NRF24_CONFIG, &config, 1);
    status = read_status();
    nrf24_read_register(NRF24_RF_CH, &rf_ch, 1);
    nrf24_read_register(NRF24_RF_SETUP, &rf_setup, 1);
    nrf24_read_register(NRF24_FIFO_STATUS, &fifo_status, 1);

    print_func("\r\n===== NRF24L01+ STATUS =====\r\n");
    print_func("CONFIG:      0x%02X\r\n", config);
    print_func("STATUS:      0x%02X\r\n", status);
    print_func("RF_CH:       %d\r\n", rf_ch);
    print_func("RF_SETUP:    0x%02X\r\n", rf_setup);
    print_func("FIFO_STATUS: 0x%02X\r\n", fifo_status);
    print_func("Mode:        %s\r\n", (config & 0x01) ? "RX" : "TX");
    print_func("Power:       %s\r\n", (config & 0x02) ? "UP" : "DOWN");
    print_func("============================\r\n");
}
