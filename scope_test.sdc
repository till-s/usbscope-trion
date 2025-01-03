#create_clock -name sdram_clk -period 6.000  [get_ports sdram_clk]
#create_clock -name ulpiClk   -period 16.667 [get_ports ulpiClk]
set_clock_groups -asynchronous -group [get_clocks sdram_*clk] -group [get_clocks ulpiClk]
set_clock_groups -asynchronous -group [get_clocks sdram_*clk] -group [get_clocks adcClk]
set_clock_groups -asynchronous -group [get_clocks adcClk] -group [get_clocks ulpiClk]

# USB3343 ULPI interface setup time min: 5.0ns, hold time min: 0.0ns.
# USB3343 ULPI interface output delay min: 1.5ns, max: 6 ns;
#  min input delay  = (data - clk trace) + 3343 min output delay + trion_delay
#  max input delay  = (data - clk trace) + 3343 max output delay + trion_delay
#  min output delay = (data + clk trace) + 3343 min hold time + trion_delay
#  max output delay = (data + clk trace) + 3343 min setup time + trion_delay
#
#  min input  delay  = 0.0 + 1.5 + trion delay
#  max input  delay  = 0.1 + 6.0 + trion delay
#  min output delay  = 0.0 + 0.0 + trion delay
#  max output delay  = 0.3 + 5.0 + trion delay
# generated ULPI IO constraints:
set_clock_latency -source -setup -2.314 [get_ports {ulpiClk}]
set_clock_latency -source -hold  -1.157 [get_ports {ulpiClk}]
set_input_delay -clock [get_clocks ulpiClk] -max  7.896 [get_ports {ulpiDir}]
set_input_delay -clock [get_clocks ulpiClk] -min  2.398 [get_ports {ulpiDir}]
set_input_delay -clock [get_clocks ulpiClk] -max  7.896 [get_ports {ulpiNxt}]
set_input_delay -clock [get_clocks ulpiClk] -min  2.398 [get_ports {ulpiNxt}]
set_output_delay -clock [get_clocks ulpiClk] -max 9.211  [get_ports {ulpiRstb}]
set_output_delay -clock [get_clocks ulpiClk] -min  1.956 [get_ports {ulpiRstb}]
set_input_delay -clock [get_clocks ulpiClk] -max  7.896 [get_ports {ulpiDat_IN[0]}]
set_input_delay -clock [get_clocks ulpiClk] -min  2.398 [get_ports {ulpiDat_IN[0]}]
set_output_delay -clock [get_clocks ulpiClk] -max 9.211 [get_ports {ulpiDat_OUT[0]}]
set_output_delay -clock [get_clocks ulpiClk] -min  1.956 [get_ports {ulpiDat_OUT[0]}]
set_input_delay -clock [get_clocks ulpiClk] -max  7.896 [get_ports {ulpiDat_IN[1]}]
set_input_delay -clock [get_clocks ulpiClk] -min  2.398 [get_ports {ulpiDat_IN[1]}]
set_output_delay -clock [get_clocks ulpiClk] -max 9.211 [get_ports {ulpiDat_OUT[1]}]
set_output_delay -clock [get_clocks ulpiClk] -min  1.956 [get_ports {ulpiDat_OUT[1]}]
set_input_delay -clock [get_clocks ulpiClk] -max  7.896 [get_ports {ulpiDat_IN[2]}]
set_input_delay -clock [get_clocks ulpiClk] -min  2.398 [get_ports {ulpiDat_IN[2]}]
set_output_delay -clock [get_clocks ulpiClk] -max 9.211 [get_ports {ulpiDat_OUT[2]}]
set_output_delay -clock [get_clocks ulpiClk] -min  1.956 [get_ports {ulpiDat_OUT[2]}]
set_input_delay -clock [get_clocks ulpiClk] -max  7.896 [get_ports {ulpiDat_IN[3]}]
set_input_delay -clock [get_clocks ulpiClk] -min  2.398 [get_ports {ulpiDat_IN[3]}]
set_output_delay -clock [get_clocks ulpiClk] -max 9.211 [get_ports {ulpiDat_OUT[3]}]
set_output_delay -clock [get_clocks ulpiClk] -min  1.956 [get_ports {ulpiDat_OUT[3]}]
set_input_delay -clock [get_clocks ulpiClk] -max  7.896 [get_ports {ulpiDat_IN[4]}]
set_input_delay -clock [get_clocks ulpiClk] -min  2.398 [get_ports {ulpiDat_IN[4]}]
set_output_delay -clock [get_clocks ulpiClk] -max 9.211 [get_ports {ulpiDat_OUT[4]}]
set_output_delay -clock [get_clocks ulpiClk] -min  1.956 [get_ports {ulpiDat_OUT[4]}]
set_input_delay -clock [get_clocks ulpiClk] -max  7.896 [get_ports {ulpiDat_IN[5]}]
set_input_delay -clock [get_clocks ulpiClk] -min  2.398 [get_ports {ulpiDat_IN[5]}]
set_output_delay -clock [get_clocks ulpiClk] -max 9.211 [get_ports {ulpiDat_OUT[5]}]
set_output_delay -clock [get_clocks ulpiClk] -min  1.956 [get_ports {ulpiDat_OUT[5]}]
set_input_delay -clock [get_clocks ulpiClk] -max  7.896 [get_ports {ulpiDat_IN[6]}]
set_input_delay -clock [get_clocks ulpiClk] -min  2.398 [get_ports {ulpiDat_IN[6]}]
set_output_delay -clock [get_clocks ulpiClk] -max 9.211 [get_ports {ulpiDat_OUT[6]}]
set_output_delay -clock [get_clocks ulpiClk] -min  1.956 [get_ports {ulpiDat_OUT[6]}]
set_input_delay -clock [get_clocks ulpiClk] -max  7.796 [get_ports {ulpiDat_IN[7]}]
set_input_delay -clock [get_clocks ulpiClk] -min  2.348 [get_ports {ulpiDat_IN[7]}]
set_output_delay -clock [get_clocks ulpiClk] -max 9.111 [get_ports {ulpiDat_OUT[7]}]
set_output_delay -clock [get_clocks ulpiClk] -min  1.905 [get_ports {ulpiDat_OUT[7]}]
set_input_delay -clock [get_clocks ulpiClk] -max  7.896 [get_ports {ulpiStp_IN}]
set_input_delay -clock [get_clocks ulpiClk] -min  2.398 [get_ports {ulpiStp_IN}]
set_output_delay -clock [get_clocks ulpiClk] -max 9.211 [get_ports {ulpiStp_OUT}]
set_output_delay -clock [get_clocks ulpiClk] -min  1.956 [get_ports {ulpiStp_OUT}]


#set_false_path -through [get_pins -regexp {.*syncReg[[]0][~]FF[|]D}]
#set_false_path -from [get_clocks adcClk] -through [get_nets -regexp {.*rWrCC.*}]
#set_false_path -from [get_clocks adcClk] -through [get_nets -regexp {.*waddrCC.*}]
#set_false_path -from [get_clocks adcClk] -through [get_nets -regexp {.*nsmplCC.*}]
#set_multicycle_path -setup -end -from [get_clocks ulpiClk] -to [get_clocks adcClk] -through [get_nets -regexp {.*[/]acqParms[.].*}] 3
# Relax hold timing
#set_false_path -hold  -from [get_clocks ulpiClk] -to [get_clocks adcClk] -through [get_nets -regexp {.*[/]acqParms[.].*}]

#set_multicycle_path -setup -end -from [get_clocks ulpiClk] -to [get_clocks adcClk] -through [get_pins -regexp {.*B2R[/]busCC.*[|]Q}] 2
#set_false_path -hold  -from [get_clocks ulpiClk] -to [get_clocks adcClk] -through [get_pins -regexp {.*B2R[/]busCC.*[|]Q}]

#set_multicycle_path -setup -end -from [get_clocks adcClk] -to [get_clocks ulpiClk] -through [get_pins -regexp {.*R2B[/]busCC.*[|]Q}] 2
#set_false_path -hold  -from [get_clocks adcClk] -to [get_clocks ulpiClk] -through [get_pins -regexp {.*R2B[/]busCC.*[|]Q}]
