library ieee;
use     ieee.math_real;

use     work.SDRAMCtrlPkg.all;
use     work.SDRAMUtilPkg.all;

entity maxAdcFreqForRam is
   generic (
      RAM_FREQ_G : natural := 0;
      ADC_BITS_G : natural := 10
   );
end entity maxAdcFreqForRam;

architecture run of maxAdcFreqForRam is
   constant NUM_CHANNELS_C : natural := 2;
   constant f : real := SDRAM_MAX_ADC_FREQ_F(real(RAM_FREQ_G), ADC_BITS_G, INSIGNIS_NDS36PT5_16ET_C, NUM_CHANNELS_C); 
begin
   process is
   begin
      wait for 1 ns;
      report "Set -gRAM_FREQ_G=<ram_freq_hz>; -gADC_BITS_G=<8/10>";
      report "Max. ADC Freq. for SDRAM clock " & integer'image(RAM_FREQ_G) & "Hz for " & integer'image(ADC_BITS_G) & "-bit ADC (" & integer'image(NUM_CHANNELS_C) & " channels) is " & real'image(f) & "Hz";
      wait;
   end process;
end architecture run;
