# PlutoSky DAB+ — plan en software

Status: in opbouw (2026-10-22). Alleen software op de pc; er is nog niets naar de Pluto gegaan of geflasht.

Doel: de PlutoSky 7020 als **DAB+-zender** (één ensemble, één of enkele diensten). De audio komt als **stereo PCM over
I2S** binnen (niet meer als MPX), de Pluto codeert zelf naar AAC, bouwt het DAB+-superframe, de multiplex en het
OFDM-signaal (modus I, Band III) en zendt het uit via de AD9361.

## 1. Licenties (open-source project, Apache-2.0)

| Onderdeel | Keuze | Waarom |
|---|---|---|
| AAC-encoder | **zelf geschreven** (deze map) | fdk-aac valt onder een eigen Fraunhofer-licentie die niet met Apache/GPL samengaat en geen octrooirechten geeft; FFmpeg (LGPL) en faac (LGPL) past niet in een Apache-project. |
| Huffman-tabellen van de AAC-norm | afgeleid uit **vo-aacenc** (VisualOn, Android, **Apache-2.0**) met bronvermelding | de codeboeken zijn normatieve gegevens uit ISO/IEC 14496-3; vo-aacenc levert ze onder dezelfde licentie als dit project. Ze worden met een script omgezet en gecontroleerd (prefixvrij, Kraft-gelijkheid). |
| Reed-Solomon, CRC's, superframe | zelf geschreven, volgens ETSI TS 102 563 | geen code overgenomen; ODR-AudioEnc is alleen als naslag gelezen. |
| Multiplex (FIC/MSC), kanaalcodering, OFDM | zelf geschreven, volgens ETSI EN 300 401 | ODR-DabMux/ODR-DabMod (GPL-3) alleen als **testreferentie** buiten de repository. |
| Testgereedschap | PyAV (FFmpeg-decoder), reedsolo | alleen om te toetsen, geen onderdeel van de software. |

**Octrooien.** Een eigen encoder lost het *auteursrecht*-vraagstuk op, niet de octrooien. Voor AAC-LC zijn de
basisoctrooien volgens algemene informatie grotendeels verlopen; voor HE-AAC (SBR, 2001–2003) en HE-AAC v2 (PS, 2004)
is dat minder zeker, en er bestaat nog een licentiepool (Via LA). Daarom: **eerst AAC-LC** (toegestaan in DAB+), SBR/PS
pas later en alleen na een octrooicheck. Dit is geen juridisch advies; voor een officiële uitzending laten toetsen.

## 2. Keten

```
I2S stereo 48 kHz ─► FPGA I2S-ontvanger ─► (DMA) ─► ARM
ARM:  [optioneel 48 → 32 kHz] ─► AAC-LC-encoder (960-punts, CPE met M/S) ─► audio-eenheden (AU)
      ─► DAB+-superframe (120 ms): header + fire code, AU's met CRC, RS(120,110) met virtuele interleaving
      ─► multiplex: FIC (FIG 0/0, 0/1, 0/2, 0/9, 0/10, 1/0, 1/1, PTy) + MSC-subkanaal, DLS-tekst als PAD
      ─► per 24 ms één CIF ─► FPGA
FPGA: energiespreiding ─► convolutiecode (K=7, R=1/4) + puncturering (EEP) ─► tijdinterleaving (16 CIF's)
      ─► frequentie-interleaving ─► QPSK + differentiële modulatie (fase-referentiesymbool)
      ─► IFFT 2048 (1536 draaggolven) ─► cyclisch prefix 504, nulsymbool 2656 ─► 2,048 MS/s
      ─► interpolatie + maskerfilter ─► AD9361 (Band III 174–240 MHz)
```

Eerste fase: de **hele keten in software op de pc**, uitvoer naar een IQ-bestand, gecontroleerd met een DAB+-decoder.
Pas daarna de verdeling ARM/FPGA.

## 3. AAC-encoder (eigen ontwerp)

| | eerste versie | later |
|---|---|---|
| Profiel | AAC-LC, framelengte **960** (verplicht in DAB+) | SBR (HE-AAC), PS (HE-AAC v2) na octrooicheck |
| Bemonstering | 48 kHz (6 AU's per superframe) en 32 kHz (4 AU's) | 24/16 kHz kern bij SBR |
| Kanalen | stereo als CPE met gezamenlijk venster, M/S per band | intensity stereo bij lage bitrates |
| Transformatie | MDCT 1920 → 960 lijnen, sinusvenster (KBD als optie) | blokwisseling met korte vensters (8 × 120) tegen pre-echo |
| Psychoakoestiek | energie per schaalfactorband, spreiding, tonaliteitsschatting, absolute drempel | verfijnen op luistertests |
| Kwantisatie | x^(3/4), globale versterking + schaalfactoren, ruis-tot-maskeringsdoel, bitreservoir per superframe | TNS |
| Entropiecodering | alle 11 spectrale codeboeken + schaalfactorcodeboek, optimale sectie-indeling | |
| Toetsing | decoderen met FFmpeg (PyAV): geen fouten, SNR/NMR tegen het origineel, luisteren | ODR-AudioEnc als kwaliteitsreferentie |

Bitrate-advies: AAC-LC stereo **96–128 kbit/s** (48 kHz) of **80–96 kbit/s** met 32 kHz kern (15 kHz audiobandbreedte,
gelijk aan FM).

## 4. Fasering

1. **Superframe + RS + CRC** (C), getoetst met reedsolo en een eigen decoder. ✔ *(2026-10-22: 24 configuraties, RS corrigeert 5 bytefouten)*
2. **AAC-LC-encoder** 960, lange vensters, M/S; toetsen met FFmpeg. ✔ *eerste versie (2026-10-22): FFmpeg decodeert foutloos,
   48 en 32 kHz, 80–128 kbit/s, niveau exact (gain 1,000), tonen S/N 54–56 dB* ← *verder: kwaliteit*
3. Blokwisseling (korte vensters), bitreservoir, kwaliteitsmetingen. ✔ *blokwisseling (2026-10-22): aanslagdetector met één
   frame vooruitkijken, LONG_START/EIGHT_SHORT/LONG_STOP, groepering rond de aanslag; pre-echo −17,7 → −45,6 dB.*
   Nog: ruisvervanging (PNS) tegen lege hoge banden bij krap budget, TNS, snelheid (nu ~30× realtime op één pc-kern;
   de ARM van de Pluto is ~10–20× trager, dus de snelheidsregeling moet efficiënter).
4. **Multiplex** (FIC/MSC) en **OFDM-modulator in C** op de pc; IQ-bestand; controle met een DAB+-ontvanger/decoder. ✔ *(2026-10-22)*
   welle-cli (welle.io, met FAAD2) ontvangt het IQ-bestand: OFDM-synchronisatie, FIC (ensemble, dienst, DAB+, subkanaal),
   superframe-synchronisatie, AAC-LC 48 kHz stereo, **DLS-tekst** (X-PAD) – S/N gelijk aan de FFmpeg-decodering.
   Op de pc kosten encoder + modulator ~1/10 van realtime; de OFDM is op de ARM te zwaar → FPGA (fase 6).
5. Cross-compileren voor de Pluto-ARM, processortijd meten. *(Vereist toestemming voor de Pluto.)*
6. Kanaalcodering + OFDM in de **FPGA**, bit-exact tegen het C-model. *(Nieuwe bitstream: vereist toestemming.)*
7. Daemon/webinterface, DAB-masker en MER via RX2, Pico in stereo-I2S-modus.

## 5. Bouwen en testen (pc)

```
make -C dab                        # bouwt de bibliotheek en de testprogramma's (gcc, WSL)
make -C dab test                   # superframe-test (Reed-Solomon via reedsolo)
build/test_mdct                    # snelle MDCT tegen de directe definitie
python tests/check_aac.py build    # coderen + decoderen met FFmpeg (PyAV), S/N en niveau
```

Testafhankelijkheden (alleen op de pc): Python met numpy, `reedsolo` en `av` (PyAV). Voor de ketentest: welle-cli (welle.io), hier
zelf gebouwd met faad2, mpg123 en LAME onder `~/dablocal` (geen root nodig).

## 7. Bestanden

| Bestand | Inhoud |
|---|---|
| `src/dabplus_sf.*`, `src/dab_rs.*`, `src/dab_crc.*` | superframe, Reed-Solomon RS(120,110), fire code en CRC |
| `src/aac_enc.*` | AAC-LC-encoder (960): psychoakoestiek, M/S, kwantisatie met snelheidsregeling, Huffman, opvulling |
| `src/aac_mdct.*` | MDCT via DCT-IV en een mixed-radix FFT (480 = 2⁵·3·5) |
| `src/aac_huff_tab.*` | codeboeken (gegenereerd, zie `NOTICE`) |
| `src/dabplus_enc.*` | PCM → superframes, verdeling van het budget over de AU's |
| `src/dab_pad.*` | Dynamic Label (DLS) in variabele X-PAD, als data stream element in de AU |
| `src/dab_mod.*` | DAB modus I: FIC (FIG 0/0, 0/1, 0/2, 0/9, 1/0, 1/1), EEP-A, energiespreiding, convolutiecode, tijd- en
  frequentie-interleaving, DQPSK, OFDM (IFFT 2048) → IQ 2,048 MS/s |
| `tests/dab_tx_file.c`, `tests/check_chain.sh` | complete zender op een bestand; ketentest met welle-cli |
| `tools/build_arm.sh` | cross-build voor de ARM van de Pluto (alleen bouwen) |
| `tools/gen_aac_huff.py` | genereert en controleert de codeboeken |
| `tests/` | `test_sf`, `check_sf.py`, `test_mdct`, `enc_file`, `check_aac.py` |

## 6. Bronnen

- ETSI TS 102 563 — DAB+ audio (superframe, RS, fire code, CRC).
- ETSI EN 300 401 — DAB (FIC, MSC, kanaalcodering, OFDM).
- ISO/IEC 14496-3 — MPEG-4 Audio (AAC).
- vo-aacenc (VisualOn/Android, Apache-2.0) — bron van de Huffman-tabellen, zie `NOTICE`.
