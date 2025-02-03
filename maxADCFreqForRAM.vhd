library ieee;
use     ieee.math_real;

use     work.SDRAMCtrlPkg.all;
use     work.SDRAMUtilPkg.all;

entity maxAdcFreqForRam is
   generic (
      RAM_FREQ_G : natural := 0;
      ADC_FREQ_G : natural := 0;
      ADC_BITS_G : natural := 10
   );
end entity maxAdcFreqForRam;

architecture run of maxAdcFreqForRam is
   constant NUM_CHANNELS_C : natural            := 2;
   constant cfg            : SDRAMDevParamsType := INSIGNIS_NDS36PT5_16ET_C;
begin
   process is
      variable f : real;
   begin
      wait for 1 ns;
      report "Set -gRAM_FREQ_G=<ram_freq_hz>; [-gADC_BITS_G=<8/10>]";
      report "or  -gADC_FREQ_G=<adc_freq_hz>; [-gADC_BITS_G=<8/10>]";
      if ( ADC_FREQ_G /= 0 ) then
         f := SDRAM_MIN_RAM_FREQ_F(real(ADC_FREQ_G), ADC_BITS_G, cfg , NUM_CHANNELS_C); 
         report "Min. RAM Freq. for ADC clock " & integer'image(ADC_FREQ_G) & "Hz for " & integer'image(ADC_BITS_G) & "-bit ADC (" & integer'image(NUM_CHANNELS_C) & " channels) is " & real'image(f) & "Hz";
      else
         f := SDRAM_MAX_ADC_FREQ_F(real(RAM_FREQ_G), ADC_BITS_G, cfg , NUM_CHANNELS_C); 
         report "Max. ADC Freq. for SDRAM clock " & integer'image(RAM_FREQ_G) & "Hz for " & integer'image(ADC_BITS_G) & "-bit ADC (" & integer'image(NUM_CHANNELS_C) & " channels) is " & real'image(f) & "Hz";
      end if;
      wait;
   end process;
end architecture run;
