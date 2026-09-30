// SPI-/GPIO-/Log-Ersatz fuer die Host-Tests der Registertreiber: bildet den
// Registersatz eines TMC nach und zeichnet jeden Schreibzugriff auf, damit
// ein Test pruefen kann, was wirklich auf dem Bus gelandet waere.
#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t  reg;       // Adresse ohne Schreibbit
    uint32_t value;
} fake_write_t;

void                fake_reset(void);            // Register, Protokoll, Log-Zaehler
void                fake_clear_log(void);        // nur das Schreibprotokoll

size_t              fake_write_count(void);
const fake_write_t *fake_write_at(size_t i);
int                 fake_writes_to(uint8_t reg);          // Anzahl Schreibzugriffe
int                 fake_last_write_index(uint8_t reg);   // -1: nie geschrieben
uint32_t            fake_reg(uint8_t reg);                // zuletzt geschriebener Wert

int                 fake_warnings(void);         // ESP_LOGW seit fake_reset()
int                 fake_errors(void);           // ESP_LOGE seit fake_reset()
