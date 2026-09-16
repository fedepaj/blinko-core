# RSLog — Protocollo ottico (v2)

## Livello fisico

- **Modulazione**: OOK (LED acceso/spento). Unità temporale: il **chip**,
  di durata `T_chip` (default 30 µs). Il chip è l'unica costante temporale:
  il ricevitore la ricava dal sync di ogni pacchetto, in righe.
- **Codifica dati**: Manchester. `bit 1 = chip 0 poi 1` (fronte di salita a
  metà bit), `bit 0 = chip 1 poi 0`. Nei dati esistono solo run di 1 o 2 chip.
- **Byte order**: MSB first.
- **Polarità**: chip `1` = LED acceso.

## Pacchetto

Tutti i pacchetti hanno la stessa lunghezza: **67 chip** (2.0 ms @ 30 µs).

```
 gap  sync                 start   id (3 bit) seed (9 bit)  payload (8 bit)  CRC8 (8 bit)
 [0]  [1 1 1 1][0 0 0 0]   [1 0]   6 chip     18 chip       16 chip          16 chip     = 67
```

- **gap** `0` e **start** `10` delimitano il sync (run di esattamente 4+4 chip,
  impossibili nei dati Manchester): tre fronti a distanza nota → stima di
  `T_chip` in righe per ogni pacchetto.
- **id** 0..7: slot messaggio. 0..5 log a rotazione, 6 = STATUS, 7 = FAULT.
- **seed** 0..511:
  - 511 = META: payload `[len:5][level:3]`, byte del messaggio grezzi;
  - 510 = META: come sopra, ma i byte sono testo impacchettato a 6 bit;
  - 509, 508 = CRC-8/ATM e CRC-8/MAXIM dell'intero messaggio (16 bit di guardia);
  - `seed < len`: pacchetto sistematico, payload = byte `seed`;
  - `len ≤ seed ≤ 507`: pacchetto **codificato**, payload = XOR dei byte
    selezionati da `rs_code_mask(seed, len)` (xorshift32, identico ovunque).
- **CRC8** (poly 0x07) sui 20 bit id+seed+payload (3 byte, ultimo nibble 0).

Livelli: 0 DEBUG, 1 INFO, 2 WARN, 3 ERROR, 4 FATAL, 5 STATUS, 6 FAULT.

## Codifica fountain

Ogni pacchetto è una riga di un sistema lineare su GF(2): il ricevitore fa
eliminazione gaussiana incrementale e ricostruisce il messaggio appena il rango
raggiunge `len`, con qualunque sottoinsieme di pacchetti (in media `len + 2`
pacchetti utili, contro `len·ln len` del protocollo a indici). I due CRC del
messaggio scartano le soluzioni corrotte da un pacchetto sfuggito al CRC-8
(probabilità residua 1/65536). I pacchetti ricevuti prima del META vengono
messi in coda (24) e riprocessati.

## Testo a 6 bit (`rs_pack.h`)

Simboli: spazio, a–z, 0–9, SHIFT (maiuscola successiva), 25 segni di
punteggiatura, ESC (byte grezzo a 8 bit). Lossless; usato solo se accorcia il
messaggio (tipicamente −20/25 %). Un messaggio da 31 byte porta fino a ~41
caratteri.

## Carosello e lampeggio visibile

Visita di un messaggio: `[META CRC CRC2] dati… [META CRC CRC2] dati…` per un
totale di `len + 6` pacchetti; i seed dei dati avanzano a ogni visita (prima
passata sistematica, poi codificata). Ordine degli slot per giro: FAULT,
STATUS, log dal più recente. Il trasmettitore alterna raffiche di `burst_on`
e pause di `burst_off` allineate ai pacchetti (default 150/50 ms): il LED
lampeggia visibilmente a 5 Hz e il ricevitore perde il 25 % del tempo.

## Canali RGB

Con un LED RGB il trasmettitore manda **tre flussi indipendenti** (R, G, B)
prelevando i pacchetti a rotazione dal carosello: tre pacchetti per intervallo
di pacchetto, sincronizzati (i sync coincidono). Ogni `pilot_ms` (100 ms) esce
un **blocco pilota** di 9·P chip (P = 8): `[buio 2P][R P][buio P][G P][buio P]
[B P][buio 2P]`. Il ricevitore lo riconosce nel profilo di luminanza (tre
impulsi uguali equispaziati fra due zone buie), misura la risposta RGB della
camera a ciascun LED (matrice 3×3, colonne normalizzate), la inverte e
"separa" i tre canali riga per riga; poi decodifica ciascun canale come un
flusso a sé. Senza piloti recenti (3 s) o con matrice mal condizionata torna
alla luminanza: una scheda con un solo LED funziona con la stessa app.
Nessuna retrocompatibilità con ricevitori solo-luminanza è prevista.

## Ricezione

Per ogni fotogramma:

1. **ROI**: colonne dove la luminanza è massima (macchia del LED sfocato).
2. **Profilo**: media della luminanza per riga nella ROI → segnale `p[r]`.
3. **Binarizzazione** con soglia locale (metà tra inviluppo min e max).
4. **Sync**: cerca run alto `L1` seguito da run basso `L2` con `L1 ≈ L2`.
   `rows_per_chip = (L1+L2)/8`.
5. **Bit**: per ciascuno dei 29 bit successivi (start + 28) confronta
   l'integrale della prima e seconda metà del bit sul profilo analogico;
   il fronte a metà bit viene usato per ri-agganciare la fase (PLL).
6. **Verifica**: start bit = 0, CRC8 ok → pacchetto valido → `rs_asm_feed`.
7. Continua la ricerca dopo il pacchetto (più pacchetti per frame se la
   macchia è alta).

Il ricevitore mantiene per ogni `id` il sistema lineare (pivot, valori), i due
CRC e la coda pre-META, ed emette il messaggio quando rango = `len` e i CRC
tornano.

## Fault record (payload dello slot 7)

Testo generato dal firmware, es.:
`HF pc=0x0000a3f2 lr=0x0000a1c1 cfsr=0x00000400` (hard fault),
`WDT reset` (reset da watchdog), `FATAL 12: sensor init` (da `RSLog.fatal`).
La causa del reset (`RSTSR0/1/2`) è inclusa nello STATUS a ogni boot.

## Parametri raccomandati (da confermare in calibrazione)

| Parametro | Valore |
|---|---|
| `T_chip` | 30 µs (16.6 kbit/s lordi; misurato su iPhone 14, vedi CALIBRATION.md) |
| esposizione telefono | minima disponibile (≤ `T_chip`/2); iPhone 14: 15 µs a 120 fps |
| righe per chip | ≥ 4 |
| lunghezza pacchetto | 67 chip = 2.0 ms |
