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
I2S stereo 48 kHz ─► FPGA I2S-ontvanger ─► audio-FIFO (1024 × {L16, R16}) ─► ARM (0x6C/0x70)
ARM (skypluto-dab): resampler 48 kHz × (1 ± ppm) naar de zenderklok ─► AAC-LC-encoder (960, CPE met M/S)
      ─► DAB+-superframe (120 ms): header + fire code, AU's met CRC, RS(120,110), DLS-tekst als X-PAD
      ─► multiplex: FIC + MSC-subkanaal, energiespreiding, convolutiecode + puncturering (EEP-A),
         tijd- en frequentie-interleaving, DQPSK ─► per draaggolf een fase-index van 2 bit (16 per 32-bit woord)
      ─► fasewoorden (7296 per 96 ms) ─► FPGA (0x68, met handshake op de bus)
FPGA: FIFO 8192 woorden ─► IFFT 2048 (1536 draaggolven) ─► cyclisch prefix 504, nulsymbool 2656 ─► 2,048 MS/s
      ─► 3/2-interpolator (72 taps) ─► 3,072 MS/s ─► × 2^gain ─► I/Q-pad van de modulator (niveau, I/Q-correctie,
      DC-nulling, zoals bij FM; kanaalfilter uit) ─► AD9361 (Band III 174–240 MHz)
```

De verdeling: alles wat bits verwerkt draait op de ARM (~1/10 van realtime op de pc), de rekenintensieve OFDM
(IFFT, 3/2-interpolatie) in de FPGA. Zo blijft de configuratie van de AD9361 en de maskerbewaking gelijk aan FM.

### Registers (WFM-blok, 0x7C440000)

| Adres | Lezen | Schrijven |
|---|---|---|
| 0x64 | DAB-besturing | [0] DAB-stand (in plaats van FM), [1] reset van het OFDM-blok, [3:2] versterking 2^n (verzadigend) |
| 0x68 | status: [13:0] vulling OFDM-FIFO, [16] onderloop (blijft tot reset), [17] CDC-FIFO vol | één fasewoord (wacht op de bus zolang de CDC-FIFO vol is, hooguit 1024 klokken) |
| 0x6C | audiosample {L[15:0], R[15:0]} (lezen haalt het uit de FIFO) | – |
| 0x70 | aantal wachtende audiosamples | – |

Het OFDM-signaal piekt rond −11 dBFS (rms −22 dBFS); versterking 1 (×2) geeft −5 dBFS piek. Het draaggolfniveau (0x08) en
de I/Q- en DC-correctie werken zoals bij FM. Na een onderloop zet `skypluto-dab` het blok in reset en begint het op een
framegrens opnieuw.

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
5. Cross-compileren voor de Pluto-ARM ✔ *(bouwt zonder waarschuwingen)*; processortijd meten op de Pluto. *(Vereist toestemming.)*
6. OFDM in de **FPGA**, bit-exact tegen het model. ✔ *(2026-10-22)* `hdl/skypluto_dab_ofdm.v`: bit-exact over twee frames,
   los gesynthetiseerd 1875 LUT, 14 BRAM, 16 DSP. **Ingebouwd in de exciter** (branch `dab-integration`, build-id B1D0001E):
   `hdl/tb_dab_top.v` stuurt de hele exciter via AXI zoals de ARM: het modulatorpad is bit-exact (0 fouten over 1,16 miljoen
   samples), de audio komt foutloos via de FIFO bij de ARM. *(Flashen: vereist toestemming.)*
7. **skypluto-dab** (ARM-programma) ✔ *eerste versie*: audio uit de FIFO, resampler met snelheidsregeling op de totale
   vertraging (PI, tijdconstante ~1 minuut, ±2000 ppm), encoder + multiplex in een eigen thread, fasewoorden met
   stroomregeling, herstart na onderloop. Simulatiestand zonder hardware (`-f`): +80 ppm klokverschil, 60 s, geen onderloop,
   welle-cli decodeert (tonen S/N ~38 dB). Nog: daemon/webinterface, DAB-masker en MER via RX2, Pico in stereo-I2S-modus.

## 5. Bouwen en testen (pc)

```
make -C dab                        # bouwt de bibliotheek en de testprogramma's (gcc, WSL)
make -C dab test                   # superframe-test (Reed-Solomon via reedsolo)
tests/check_chain.sh in.f32        # dab_tx_file → welle-cli
tests/check_app.sh [ppm] [s]       # skypluto-dab in simulatiestand → FPGA-model → welle-cli
python3 hdl/check_hdl.py build/ph.bin 2         # OFDM-blok tegen het model (iverilog)
TOP=1 python3 hdl/check_hdl.py build/ph.bin 2   # de hele exciter in DAB-stand (tb_dab_top.v)
build/test_mdct                    # snelle MDCT tegen de directe definitie
python tests/check_aac.py build    # coderen + decoderen met FFmpeg (PyAV), S/N en niveau
```

Testafhankelijkheden (alleen op de pc): Python met numpy, `reedsolo` en `av` (PyAV). Voor de ketentest: welle-cli (welle.io), hier
zelf gebouwd met faad2, mpg123 en LAME onder `~/dablocal` (geen root nodig).
`TONE=1000 tests/check_app.sh 0 20` toetst de ingebouwde testtoon (`-T`).

### Eerste test op de Pluto

```
dab/tools/install_dab_test.sh <ip> out/dab/BOOT.bin   # zet alles klaar, verandert niets aan wat er opstart
ssh root@<ip> /mnt/jffs2/dab-boot.sh dab              # BOOT.bin.dab opstarten (FM blijft werken, + DAB)
ssh root@<ip> /mnt/jffs2/dab-test.sh 174928000        # kanaal 5A, testtoon 1 kHz links / 1,5 kHz rechts; Ctrl-C = terug naar FM
ssh root@<ip> /mnt/jffs2/dab-boot.sh fm               # terug naar de FM-release (BOOT.bin.fm)
```

`dab-test.sh` zet via de daemon de TX-bandbreedte op 3 MHz en stemt af (met kalibratie); na afloop gaat de bandbreedte terug.
Het DAB-signaal ligt ~16 dB onder de FM-draaggolf bij dezelfde P-instelling; de maskerbewaking (FM-masker) zegt tijdens de
test niets. `dab-test.sh <f> i2s` gebruikt de I2S-ingang (stereo 48 kHz) in plaats van de toon.

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
| `app/skypluto_dab.c` | het zendprogramma op de ARM (registers, resampler, snelheidsregeling, threads; `-f` = simulatie) |
| `tests/check_app.sh` | skypluto-dab in simulatie → FPGA-model → welle-cli |
| `hdl/` | `skypluto_dab_ofdm.v` (kopie in `hdl/library/skypluto_wfm`), model, generatoren, testbanken, OOC-synthese |
| `tools/build_arm.sh` | cross-build voor de ARM van de Pluto (alleen bouwen) |
| `tools/install_dab_test.sh`, `tools/dab-boot.sh`, `tools/dab-test.sh` | DAB-test op de Pluto: klaarzetten, opstartkeuze FM/DAB, test met toon of I2S |
| `tools/gen_aac_huff.py` | genereert en controleert de codeboeken |
| `tests/` | `test_sf`, `check_sf.py`, `test_mdct`, `enc_file`, `check_aac.py` |

## 6. Bronnen

- ETSI TS 102 563 — DAB+ audio (superframe, RS, fire code, CRC).
- ETSI EN 300 401 — DAB (FIC, MSC, kanaalcodering, OFDM).
- ISO/IEC 14496-3 — MPEG-4 Audio (AAC).
- vo-aacenc (VisualOn/Android, Apache-2.0) — bron van de Huffman-tabellen, zie `NOTICE`.
