#!/bin/sh
# =============================================================================
# skypluto-wfm.sh - WFM exciter control on the PlutoSky (via ssh, busybox)
# -----------------------------------------------------------------------------
# Requirements: (1) always a carrier on a chosen frequency, (2) adjustable
# offset to zero-IF, (3) adjustable carrier level.
#
#   frequency = AD9361-LO (iio)         -> skypluto-wfm.sh freq  <Hz>
#   offset    = digitale NCO (AXI-reg)  -> skypluto-wfm.sh offset <Hz>   (LO+offset)
#   level     = digital (AXI reg)      -> skypluto-wfm.sh level  <0..65535>
#             + AD9361 TX-atten (iio)   -> skypluto-wfm.sh atten  <dB 0..89>
#
# Requires on the Pluto: devmem, iio_attr, iio_writedev (libiio-utils).
# =============================================================================
set -e

# ---- config ----------------------------------------------------------------
PHY="ad9361-phy"
DDS="cf-ad9361-dds-core-lpc"       # DAC/DDS core (name may differ slightly)
RATE=3072000                        # iio TX-baseband-samplerate
LCLK=12288000                       # ACTUAL fabric clock (l_clk) = 4x RATE, measured on the board.
                                    # The phase accumulator (offset + deviation) runs on l_clk!
FREQ_DEFAULT=98200000               # 98.2 MHz - ADJUST to an allowed frequency
ATTEN_DEFAULT=10                    # dB TX attenuation (10 dB + full digital = ~-5 dBm)

# AXI-Lite registers (basis 0x7C440000)
R_CTRL=0x7C440000                   # [0]=enable
R_OFFSET=0x7C440004                 # NCO phase-increment (signed 24-bit)
R_LEVEL=0x7C440008                  # carrier-level 0..65535
R_KDEV=0x7C44000C                   # FM-deviatie-gain
R_STAT=0x7C440010                   # ro

TWO24=16777216                      # 2^24 (PHASE_W)

# ---- helpers ---------------------------------------------------------------
# NB: TX channels are OUTPUT -> always '-o' (otherwise iio_attr does not find them).
lo()    { iio_attr -q -o -c "$PHY" altvoltage1 frequency "$1" >/dev/null; }
rate()  { iio_attr -q -o -c "$PHY" voltage0 sampling_frequency "$1" >/dev/null; }
atten() { iio_attr -q -o -c "$PHY" voltage0 hardwaregain "-$1" >/dev/null; }

# offset in Hz -> phase increment = round(Hz * 2^24 / l_clk), 32-bit two's complement
# (l_clk = 12.288 MHz, NOT the baseband rate - the phase accumulator runs on l_clk)
offset_hz() {
    inc=$(( $1 * TWO24 / LCLK ))
    val=$(( inc & 0xFFFFFFFF ))
    devmem $R_OFFSET 32 $val
}

# Set the DAC source to "external/DMA" (dac_data_sel=2) so the core drives OUR fabric IQ
# instead of the internal DDS, AND keep the DMA "alive" so the starve-mute
# does not trigger: start a cyclic null buffer.
# (Bring-up step; verify device name/behaviour on the board.)
enable_fabric_tx() {
    # Cyclic null buffer in the background -> driver sets dac_data_sel=2 (core
    # uses our fabric IQ) AND keeps the DMA "alive" so the starve-mute does not
    # trigger. The zeros themselves go nowhere (we drive dac_data ourselves).
    # NOTE: pkill returns exit 1 when nothing matches -> under 'set -e' that would
    # abort the script (nothing is running yet at boot). Hence '|| true'.
    pkill -f "iio_writedev.*$DDS" 2>/dev/null || true
    nohup sh -c "iio_writedev -c -b 8192 '$DDS' voltage0 voltage1 < /dev/zero" \
        >/tmp/wfm_tx.log 2>&1 &
    sleep 1
    echo "  fabric-TX geactiveerd (dac_data_sel=2 via cyclische buffer)"
}
disable_fabric_tx() { pkill -f "iio_writedev.*$DDS" 2>/dev/null || true; }

# ---- commands ------------------------------------------------------------
case "$1" in
  freq)    lo "$2";                      echo "LO = $2 Hz" ;;
  rate)    rate "$2";                    echo "rate = $2 SPS" ;;
  atten)   atten "$2";                   echo "atten = $2 dB" ;;
  offset)  offset_hz "$2";               echo "offset = $2 Hz (reg $(devmem $R_OFFSET))" ;;
  level)   devmem $R_LEVEL 32 "$2";      echo "level = $2" ;;
  kdev)    devmem $R_KDEV 32 "$2";       echo "kdev = $2" ;;
  enable)  devmem $R_CTRL 32 1;          echo "enable = 1" ;;
  disable) devmem $R_CTRL 32 0;          echo "enable = 0" ;;
  start)
    F=${2:-$FREQ_DEFAULT}; A=${3:-$ATTEN_DEFAULT}
    echo "WFM START  LO=$F Hz  rate=$RATE  atten=$A dB"
    rate $RATE; lo "$F"; atten "$A"
    devmem $R_LEVEL 32 65535     # full
    devmem $R_CTRL  32 1         # enable
    enable_fabric_tx
    echo "draaggolf actief op $F Hz (+ offset). Stel bij met: offset/level/atten/kdev."
    ;;
  stop)    atten 89; iio_attr -q -o -c "$PHY" altvoltage1 powerdown 1 >/dev/null 2>&1 || true; disable_fabric_tx; devmem $R_CTRL 32 0; echo "TX gedempt + TX-LO uit" ;;
  status)
    echo "LO     : $(iio_attr -o -c $PHY altvoltage1 frequency 2>/dev/null) Hz"
    echo "rate   : $(iio_attr -o -c $PHY voltage0 sampling_frequency 2>/dev/null) SPS"
    echo "atten  : $(iio_attr -o -c $PHY voltage0 hardwaregain 2>/dev/null)"
    echo "ctrl   : $(devmem $R_CTRL)   offset: $(devmem $R_OFFSET)"
    echo "level  : $(devmem $R_LEVEL)  kdev: $(devmem $R_KDEV)"
    echo "status : $(devmem $R_STAT)"
    ;;
  *)
    echo "gebruik: $0 {start [Hz] [atten]|stop|freq Hz|offset Hz|level 0..65535|atten dB|kdev n|enable|disable|status}"
    exit 1 ;;
esac
