library ieee;
use     ieee.math_real.all;

use     work.SDRAMCtrlPkg.all;

package SDRAMUtilPkg is

   -- Functions to check compatibility of SDRAM devices
   -- with the chosen ADC frequency, bit-size and RAM frequency.

   -- Parameters of ram devices unknown to SDRAMCtrlPkg could
   -- be defined here as well...

   -- Note that only 16-bit wide SDRAMs are supported by
   -- SampleBuffer implementation.

   -- TIMING PARAMETERS:
   --
   -- The maximal achievable ADC clock frequency depends
   -- on the
   --   max. SDRAM clock frequency
   --   SDRAM timing parameters
   --   number of ADC bits (when using a 10-bit ADC
   --   the RAM must be clocked faster since only
   --   16-bits can be stored per SDRAM clock cycle
   --   while the ADC produces 20 bits per ADC clock.
   --
   -- SDRAM bandwidth is reduced due to
   --
   --   a) Periodic refresh cycles; to maintain
   --      a write-clock speed of F_ADC the SDRAM
   --      clock speed must be increased by a factor
   --
   --      (1 + (T_RP_C + T_RFC)/(T_REF_C/2**SDRAM_LD_ROWS_C))
   --
   --   b) Bank-switching/precharging. Assuming the write-
   --      address always increases in a linear fashion
   --      (this is the case for our FIFO implementation)
   --      implies that switching rows within a single bank
   --      (a more time-consuming operation) can never happen.
   --
   --      The required increase in SDRAM clock frequency
   --      computes as:
   --
   --      (1 + min(ceil(T_RCD_C * CLK_FREQ + 1),2))/2**SDRAM_LD_COLS_C)
   --
   --      The SDRAM controller needs at least 3 cycles
   --      (1 in addition to the active->write delay if the latter
   --      exceeds 2 cycles) to switch banks (different rows).
   --
   --   c) Data "stuffing": if the ADC data width is
   --      wider than the SDRAM data port (16 bits) then
   --      the RAM must run faster in order to store
   --      the data produced by the ADC. Only 8- or 10-bit
   --      ADCs are supported ATM. An 8-bit device (with
   --      two channels) neatly feeds a 16-bit SDRAM. OTOH,
   --      two channels @10-bits each produce 20 bits and
   --      require a SDRAM clock increase by
   --
   --      20/16 = 5/4

   function SDRAM_MAX_ADC_FREQ_F(
      constant ramClockFreqHz : in  real;
      constant sampleSizeBits : in  natural;
      constant deviceParams   : in  SDRAMDevParamsType;
      constant numChannels    : in  natural := 2
   ) return real;

   function SDRAM_FREQUENCY_CHECK_F(
      constant adcClockFreqHz : in  real;
      constant ramClockFreqHz : in  real;
      constant sampleSizeBits : in  natural;
      constant deviceParams   : in  SDRAMDevParamsType;
      constant numChannels    : in  natural := 2
   ) return boolean;

end package SDRAMUtilPkg;

package body SDRAMUtilPkg is

   function SDRAM_MAX_ADC_FREQ_F(
      constant ramClockFreqHz : in  real;
      constant sampleSizeBits : in  natural;
      constant deviceParams   : in  SDRAMDevParamsType;
      constant numChannels    : in  natural := 2
   )
   return real is
      variable maxFreq : real;
   begin
      if ( ramClockFreqHz > deviceParams.CLK_FREQ_MAX ) then
         return 0.0;
      end if;
      maxFreq := ramClockFreqHz * (16.0/real(numChannels*sampleSizeBits));
      maxFreq := maxFreq / (1.0 + (realmin(ceil(deviceParams.T_RCD * ramClockFreqHz),2.0) + 1.0)/2.0**deviceParams.C_WIDTH);
      maxFreq := maxFreq / (1.0 + (deviceParams.T_RP + deviceParams.T_RFC)/(deviceParams.T_REF/2.0**deviceParams.R_WIDTH));
      return maxFreq;
   end function SDRAM_MAX_ADC_FREQ_F;

   function SDRAM_FREQUENCY_CHECK_F(
      constant adcClockFreqHz : in  real;
      constant ramClockFreqHz : in  real;
      constant sampleSizeBits : in  natural;
      constant deviceParams   : in  SDRAMDevParamsType;
      constant numChannels    : in  natural := 2
   )
   return boolean is
   begin
      if ( ramClockFreqHz > deviceParams.CLK_FREQ_MAX ) then
         return false;
      end if;
      return adcClockFreqHz < SDRAM_MAX_ADC_FREQ_F(ramClockFreqHz, sampleSizeBits, deviceParams, numChannels);
   end function SDRAM_FREQUENCY_CHECK_F;

end package body SDRAMUtilPkg;
