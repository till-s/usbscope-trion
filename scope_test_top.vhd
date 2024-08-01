library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;

use     work.UlpiPkg.all;
use     work.Usb2Pkg.all;
use     work.Usb2UtilPkg.all;
use     work.Usb2AppCfgPkg.all;
use     work.Usb2DescPkg.all;
use     work.Usb2MuxEpCtlPkg.all;
use     work.Usb2EpGenericCtlPkg.all;
use     work.Usb2AppCfgPkg.all;
use     work.CommandMuxPkg.all;
use     work.BasicPkg.Slv8Array;
use     work.SDRAMPkg.all;

entity scope_test_top is
   generic (
      USE_SMPL_CLK_G    : boolean := false;
      -- pipeline stages in the readback:
      --   external registers (in IO ring) (1 in out, 1 in in direction)
      --   sampling and resync to sdram_clk; the precise value also depends
      --   on the fine-tuning of the sdram_smpl_clk phase...
      SDRAM_READ_DLY_G  : natural := 3;
      GIT_VERSION_G     : std_logic_vector(31 downto 0) := x"0000_0000";
      USE_SDRAM_BUF_G   : boolean := true;
      ADC_FREQ_G        : real    := 130.0E6;
      USE_SIN_GEN_G     : boolean := false;
      NO_DECIMATORS_G   : boolean := false;
      HAVE_SPI_CMD_G    : boolean := true;
      ADC_BITS_G        : natural := 10
   );
   port (
      sdram_clk         : in  std_logic;
      sdram_smpl_clk    : in  std_logic;
      -- create 180-deg out of phase clock with DDIO
      sdram_clkout_HI   : out std_logic := '1';

      sdram_CSb         : out std_logic;
      sdram_CKE         : out std_logic;
      sdram_DQML        : out std_logic;
      sdram_DQMH        : out std_logic;
      sdram_WEb         : out std_logic;
      sdram_CASb        : out std_logic;
      sdram_RASb        : out std_logic;
      sdram_A           : out std_logic_vector(12 downto 0);
      sdram_BA          : out std_logic_vector( 1 downto 0);
      sdram_DQ_IN       : in  std_logic_vector(15 downto 0);
      sdram_DQ_OUT      : out std_logic_vector(15 downto 0);
      sdram_DQ_OE       : out std_logic_vector(15 downto 0);
      ulpiClk           : in    std_logic;
      -- NOTE    : unfortunately, the ulpiClk stops while ulpiRstb is asserted...
      ulpiRstb          : out   std_logic                    := '1';
      ulpiDat_IN        : in    std_logic_vector(7 downto 0) := (others => '0');
      ulpiDat_OUT       : out   std_logic_vector(7 downto 0) := (others => '0');
      ulpiDat_OE        : out   std_logic_vector(7 downto 0) := (others => '0');
      ulpiDir           : in    std_logic                    := '0';
      ulpiNxt           : in    std_logic                    := '0';
      ulpiStp_IN        : in    std_logic                    := '0';
      ulpiStp_OUT       : out   std_logic                    := '0';
      ulpiStp_OE        : out   std_logic                    := '0';
      ulpiPllLocked     : in    std_logic;
      sdramPllLocked    : in    std_logic;

      adcClk            : in    std_logic;
      ADC_DDR_HI        : in    std_logic_vector(ADC_BITS_G downto 0);
      ADC_DDR_LO        : in    std_logic_vector(ADC_BITS_G downto 0);

      spiSClk           : out   std_logic;
      spiMOSI           : out   std_logic;
      spiMISO           : in    std_logic;
      spiCSb_OUT        : out   std_logic;
      spiCSb_OE         : out   std_logic := '1';
      spiCSb_IN         : in    std_logic
   );
end entity scope_test_top;

architecture rtl of scope_test_top is
   attribute ASYNC_REG         : string;
   attribute SYN_PRESERVE      : boolean;

   component CosGen is
      port (
         clk      : in  std_logic;
         load     : in  std_logic;
         coeff    : in  signed(17 downto 0);
         aini     : in  signed(34 downto 0);
         phasCos  : in  boolean;
         cos      : out signed(34 downto 0)
      );
   end component CosGen;


   function toSlv(constant a : in Slv8Array)
   return std_logic_vector is
      variable v : std_logic_vector(8*a'length - 1 downto 0);
   begin
      for i in a'low to a'high loop
         v(8*(i - a'low) + 7 downto 8*(i - a'low)) := a(i);
      end loop;
      return v;
   end function toSlv;

   -- must cover bulk max pkt size
   constant LD_FIFO_OUT_C      : natural :=  9;
   constant LD_FIFO_INP_C      : natural :=  9;
   constant SDRAM_A_WIDTH_C    : natural := 13; -- row
   constant SDRAM_B_WIDTH_C    : natural :=  2; -- bank
   constant SDRAM_C_WIDTH_C    : natural :=  9; -- bank
   constant FLAT_A_WIDTH_C     : natural := SDRAM_A_WIDTH_C + SDRAM_B_WIDTH_C + SDRAM_C_WIDTH_C;
   constant NSMPL_MAX_C        : natural := (2**FLAT_A_WIDTH_C * 4)/5;

   constant NUM_REGS_C         : natural := 20;

   constant MEM_DEPTH_C        : natural := ite( USE_SDRAM_BUF_G, 3354624, 2048 );

   signal acmFifoOutDat        : Usb2ByteType;
   signal acmFifoOutEmpty      : std_logic;
   signal acmFifoOutRen        : std_logic    := '1';
   signal acmFifoOutVld        : std_logic;
   signal acmFifoInpDat        : Usb2ByteType := (others => '0');
   signal acmFifoInpFull       : std_logic;
   signal acmFifoInpWen        : std_logic    := '0';

   signal bbi                  : std_logic_vector(7 downto 0);
   signal bbo                  : std_logic_vector(7 downto 0);
   signal subCmdBB             : SubCommandBBType;

   signal acmFifoInpMinFill    : unsigned(LD_FIFO_INP_C - 1 downto 0) := (others=> '0');
   signal acmFifoInpTimer      : unsigned(32 - 1 downto 0) := (others=> '0');

   signal acmFifoLocal         : std_logic    := '1';

   signal acmDTR               : std_logic;
   signal acmRate              : unsigned(31 downto 0);
   signal acmParity            : unsigned( 2 downto 0);
   signal acmFifoRst           : std_logic    := '0';

   signal usb2Rst              : std_logic    := '0';
   signal usb2DevStatus        : Usb2DevStatusType := USB2_DEV_STATUS_INIT_C;

   signal sdramBusReq          : SDRAMReqType := SDRAM_REQ_INIT_C;
   signal sdramBusRep          : SDRAMRepType := SDRAM_REP_INIT_C;
   signal sdramBusRVldDly      : std_logic_vector(SDRAM_READ_DLY_G - 1 downto 0) := (others => '0');
   signal sdramBusRVld         : std_logic    := '0';

   signal sdramDQOE            : std_logic;

   signal ulpiIb               : UlpiIbType := ULPI_IB_INIT_C;
   signal ulpiOb               : UlpiObType := ULPI_OB_INIT_C;
   signal ulpiRx               : UlpiRxType;
   signal ulpiRst              : std_logic := '0';
   signal ulpiForceStp         : std_logic := '0';
   signal usb2HiSpeedEn        : std_logic := '1';
   signal ulpiDirB             : std_logic;

   signal fifoRDat             : Usb2ByteType;
   signal fifoRRdy             : std_logic;
   signal fifoRVld             : std_logic;
   signal fifoWDat             : Usb2ByteType;
   signal fifoWRdy             : std_logic;
   signal fifoWVld             : std_logic;

   signal syncDQIn             : std_logic_vector(15 downto 0);

   signal adcDatA              : std_logic_vector(ADC_BITS_G downto 0) := (others => '0');
   signal adcDatB              : std_logic_vector(ADC_BITS_G downto 0) := (others => '0');
   signal adcDatAReg           : std_logic_vector(ADC_BITS_G downto 0) := (others => '0');
   signal adcDatBReg           : std_logic_vector(ADC_BITS_G downto 0) := (others => '0');
   signal adcCnt               : signed(ADC_BITS_G - 1 downto 0)       := (others => '0');

   signal regRDat              : std_logic_vector(7 downto 0) := (others => '0');
   signal regWDat              : std_logic_vector(7 downto 0);
   signal regAddr              : unsigned(7 downto 0);
   signal regRdnw              : std_logic;
   signal regVld               : std_logic;
   signal regRdy               : std_logic := '1';
   signal regErr               : std_logic := '1';
   signal regWen               : std_logic;
   signal genLoad              : std_logic := '0';
   signal load                 : std_logic := '0';

   subtype RegIdxType          is integer range 0 to NUM_REGS_C - 1;

   signal regs                 : Slv8Array(RegIdxType) := ( 18 => x"01", others => (others => '0') );
   signal regIdx               : RegIdxType;

   signal adcCos               : signed(34 downto 0);
   signal adcSin               : signed(34 downto 0);
   signal adcASini             : signed(34 downto 0) := to_signed(112233, 35);
   signal adcACini             : signed(34 downto 0) := to_signed(11223344, 35);
   signal adcCoeff             : signed(17 downto 0);
   signal adcCini              : signed(17 downto 0);
   signal adcAini              : signed( 7 downto 0);

begin

   G_SYNC : if ( USE_SMPL_CLK_G ) generate
      G_SYNC_BIT : for i in sdram_DQ_IN'range generate
         signal syncBit : std_logic_vector(1 downto 0);
         attribute ASYNC_REG of syncBit : signal is "TRUE";
      begin

         P_SYNC_SMP : process ( sdram_smpl_clk ) is
         begin
            if ( rising_edge( sdram_smpl_clk ) ) then
               syncBit(1) <= sdram_DQ_IN(i);
            end if;
         end process P_SYNC_SMP;

         P_SYNC_REG : process ( sdram_clk ) is
         begin
            if ( rising_edge( sdram_clk ) ) then
               syncBit(0)       <= syncBit(1);
            end if;
         end process P_SYNC_REG;

         syncDQIn(i) <= syncBit(0);

      end generate G_SYNC_BIT;
   end generate G_SYNC;
   
   G_NO_SYNC : if ( not USE_SMPL_CLK_G ) generate
   begin
      P_IN_REG : process ( sdram_clk ) is
      begin
         if ( rising_edge( sdram_clk ) ) then
            syncDQIn <= sdram_DQ_IN;
         end if;
      end process P_IN_REG;
   end generate G_NO_SYNC;
   
   P_DLY_REG : process ( sdram_clk ) is
   begin
      if ( rising_edge( sdram_clk ) ) then
         -- account for the read-back delay in the external IO register and
         -- this resynchronizing stage
         sdramBusRVldDly  <= sdramBusRVld & sdramBusRVldDly(sdramBusRVldDly'left downto 1);
      end if;
   end process P_DLY_REG;

   sdramBusRep.vld <= sdramBusRVldDly(0);

   P_INI : process ( ulpiClk ) is
      variable cnt : unsigned(29 downto 0)        := (others => '1');
      variable rst : std_logic_vector(3 downto 0) := (others => '1');
      attribute ASYNC_REG of rst : variable is "TRUE";

   begin
      if ( rising_edge( ulpiClk ) ) then
         if ( cnt( cnt'left ) = '1' ) then
            cnt := cnt - 1;
         end if;
         rst := not ulpiPllLocked & rst(rst'left downto 1);
      end if;
      ulpiRst      <= rst(0);
      usb2Rst      <= rst(0);
   end process P_INI;

   acmFifoOutVld <= not acmFifoOutEmpty;

   U_BUF : entity work.Usb2StrmBuf
      port map (
         clk   => ulpiClk,
         rst   => acmFifoRst,

         vldIb => acmFifoOutVld,
         rdyIb => acmFifoOutRen,
         datIb => acmFifoOutDat,

         vldOb => fifoRVld,
         rdyOb => fifoRRdy,
         datOb => fifoRDat
      );

   acmFifoInpDat <= fifoWDat;
   acmFifoInpWen <= fifoWVld;
   fifoWRdy      <= not acmFifoInpFull;

   U_CMD : entity work.CommandWrapper
   generic map (
      GIT_VERSION_G                => GIT_VERSION_G,
      FIFO_FREQ_G                  => 60.0E6,
      ADC_FREQ_G                   => ADC_FREQ_G,
      ADC_BITS_G                   => ADC_BITS_G,
      RAM_BITS_G                   => ADC_BITS_G,
      MEM_DEPTH_G                  => MEM_DEPTH_C,
      SDRAM_ADDR_WIDTH_G           => FLAT_A_WIDTH_C,
      USE_SDRAM_BUF_G              => USE_SDRAM_BUF_G,
      HAVE_SPI_CMD_G               => HAVE_SPI_CMD_G,
      HAVE_REG_CMD_G               => true,
      HAVE_BB_CMD_G                => true,
      DISABLE_DECIMATORS_G         => NO_DECIMATORS_G,
      REG_ASYNC_G                  => true
   )
   port map (
      clk                          => ulpiClk,
      rst                          => acmFifoRst,

      datIb                        => fifoRDat,
      vldIb                        => fifoRVld,
      rdyIb                        => fifoRRdy,
      datOb                        => fifoWDat,
      vldOb                        => fifoWVld,
      rdyOb                        => fifoWRdy,

      bbo                          => bbo, --: out std_logic_vector(7 downto 0);
      bbi                          => bbi, --: in  std_logic_vector(7 downto 0) := (others => '0');
      subCmdBB                     => subCmdBB, --: out SubCommandBBType;

      adcStatus                    => open, --: out std_logic_vector(7 downto 0) := (others => '0');
      err(0)                       => open,
      err(1)                       => open,

      -- register interface
      regClk                       => adcClk,
      regRDat                      => regRDat, --: in  std_logic_vector(7 downto 0) := (others => '0');
      regWDat                      => regWDat, --: out std_logic_vector(7 downto 0);
      regAddr                      => regAddr, --: out unsigned(7 downto 0);
      regRdnw                      => regRdnw, --: out std_logic;
      regVld                       => regVld,  --: out std_logic;
      regRdy                       => regRdy,  --: in  std_logic := '1';
      regErr                       => regErr,  --: in  std_logic := '1'

      spiSClk                      => spiSClk, --: out std_logic;
      spiMOSI                      => spiMOSI, --: out std_logic;
      spiCSb                       => spiCSb_OUT,  --: out std_logic;
      spiMISO                      => spiMISO, --: in  std_logic := '0';

      adcClk                       => adcClk,
      adcRst                       => open,

      -- bit 0 is the DOR (overrange) bit
      adcDataA                     => adcDatAReg,
      adcDataB                     => adcDatBReg,

      extTrg                       => open, --: in  std_logic := '0';

      -- SDRAM interface
      sdramClk                     => sdram_clk,
      sdramReq                     => sdramBusReq,
      sdramRep                     => sdramBusRep
   );

   U_USB_DEV : entity work.Usb2ExampleDev
      generic map (
         ULPI_CLK_MODE_INP_G       => false,
         DESCRIPTORS_G             => USB2_APP_DESCRIPTORS_C,
         DESCRIPTORS_BRAM_G        => true,
         LD_ACM_FIFO_DEPTH_INP_G   => LD_FIFO_INP_C,
         LD_ACM_FIFO_DEPTH_OUT_G   => LD_FIFO_OUT_C,
         CDC_ACM_ASYNC_G           => false,
         ULPI_EMU_MODE_G           => NONE,
         MARK_DEBUG_ULPI_IO_G      => false,
         MARK_DEBUG_PKT_TX_G       => false,
         MARK_DEBUG_PKT_RX_G       => false,
         MARK_DEBUG_PKT_PROC_G     => false
      )
      port map (
         usb2Clk                   => ulpiClk,
         usb2Rst                   => usb2Rst,
         usb2RstOut                => open,
         ulpiRst                   => ulpiRst,
         ulpiIb                    => ulpiIb,
         ulpiOb                    => ulpiOb,
         ulpiRx                    => ulpiRx,
         ulpiForceStp              => ulpiForceStp,

         usb2HiSpeedEn             => usb2HiSpeedEn,

         usb2DevStatus             => usb2DevStatus,

         acmFifoClk                => ulpiClk,
         acmFifoOutDat             => acmFifoOutDat,
         acmFifoOutEmpty           => acmFifoOutEmpty,
         acmFifoOutRen             => acmFifoOutRen,
         acmFifoInpDat             => acmFifoInpDat,
         acmFifoInpFull            => acmFifoInpFull,
         acmFifoInpWen             => acmFifoInpWen,

         acmFifoInpMinFill         => acmFifoInpMinFill,
         acmFifoInpTimer           => acmFifoInpTimer,
         acmFifoLocal              => acmFifoLocal,

         acmDTR                    => acmDTR,
         acmRate                   => acmRate,
         acmParity                 => acmParity
      );

   G_SDRAM_CTRL : if ( USE_SDRAM_BUF_G ) generate

      U_SDRAM : entity work.SDRAMCtrl
         generic map (
            EXT_OUT_REG_G             => false,
            INP_REG_G                 => 2,
            CLK_FREQ_G                => 166.0E6,
            A_WIDTH_G                 => SDRAM_A_WIDTH_C,
            B_WIDTH_G                 => SDRAM_B_WIDTH_C,
            C_WIDTH_G                 => SDRAM_C_WIDTH_C
         )
         port map (
            clk                       => sdram_clk,
            req                       => sdramBusReq.req,
            rdnwr                     => sdramBusReq.rdnwr,
            ack                       => sdramBusRep.ack,
            addr                      => sdramBusReq.addr(FLAT_A_WIDTH_C - 1 downto 0),
            wdat                      => sdramBusReq.wdat,
            wstrb                     => open,
            rdat                      => sdramBusRep.rdat,
            vld                       => sdramBusRVld,
            rdy                       => sdramBusRep.rdy,
            sdramCSb                  => sdram_CSb,
            sdramCKE                  => sdram_CKE,
            sdramDQM(0)               => sdram_DQML,
            sdramDQM(1)               => sdram_DQMH,
            sdramWEb                  => sdram_WEb,
            sdramCASb                 => sdram_CASb,
            sdramRASb                 => sdram_RASb,
            sdramAddr                 => sdram_A(SDRAM_A_WIDTH_C - 1 downto 0),
            sdramBank                 => sdram_BA,
            sdramDQInp                => syncDQIn,
            sdramDQOut                => sdram_DQ_OUT,
            sdramDQOE                 => sdramDQOE
         );

   end generate G_SDRAM_CTRL;

   sdram_A(sdram_A'left downto SDRAM_A_WIDTH_C) <= ( others => '0' );

   sdram_DQ_OE   <= (others => sdramDQOE);

   ulpiDat_OUT   <= ulpiOb.dat;
   ulpiIb.dat    <= ulpiDat_IN;
   ulpiDat_OE    <= (others => ulpiDirB);

   ulpiDirB      <= not ulpiDir;
   ulpiIb.dir    <= ulpiDir;

   ulpiStp_OUT   <= ulpiOb.stp;
   ulpiIb.stp    <= ulpiStp_IN;
   ulpiStp_OE    <= '1';

   ulpiIb.nxt    <= ulpiNxt;

   P_SIM : process ( adcClk ) is
   begin
      if ( rising_edge( adcClk ) ) then
         if ( adcCnt = -2 ) then
            adcCnt <= (others => '0');
         else
            adcCnt <= adcCnt + 1;
         end if;
         adcDatAReg <= adcDatA;
         adcDatBReg <= adcDatB;
      end if;
   end process P_SIM;

   G_ADC_CNT : if ( not USE_SIN_GEN_G ) generate

   adcDatA <= ADC_DDR_HI;
   adcDatB <= ADC_DDR_LO;

   end generate G_ADC_CNT;


   process ( ulpiClk ) is
      variable cnt : unsigned(25 downto 0) := (others => '0');
   begin
      if ( rising_edge( ulpiClk ) ) then
         cnt := cnt + 1;
      end if;
   end process;
   
   process ( regAddr, regIdx ) is
   begin
      regErr  <= '1';
      regRdy  <= '1';
      regIdx  <= 0;
      if ( regAddr < regs'length ) then
         regErr  <= '0';
         regIdx  <= to_integer( regAddr );
      end if;
      regRDat <= regs( regIdx );
   end process;

   regWen <= (regVld and not regErr and not regRdnw);

   process ( adcClk ) is
   begin
      if ( rising_edge( adcClk ) ) then
         genLoad <= regWen;
         if (  regWen = '1' ) then
            regs( regIdx ) <= regWDat;
         end if;
      end if;
   end process;

   G_ADC_SIN : if ( USE_SIN_GEN_G ) generate

--   U_SIN_COS : entity work.SinCosGen
--      generic map (
--         MULTIPLIER_G => "EFX"
--      )
--      port map (
--         clk      => adcClk,
--         load     => load,
--         coeff    => adcCoeff,
--         cini     => adcCini,
--         aini     => adcAini,
--         cos      => adcCos,
--         sin      => adcSin
--      );

      U_COS : CosGen
         port map (
            clk      => adcClk,
            load     => load,
            coeff    => adcCoeff,
            aini     => adcACini,
            phasCos  => true,
            cos      => adcCos
         );

      U_SIN : CosGen
         port map (
            clk      => adcClk,
            load     => load,
            coeff    => adcCoeff,
            aini     => adcASini,
            phasCos  => false,
            cos      => adcSin
         );

      P_REG_PARMS : process ( adcClk ) is
      begin
         if ( rising_edge( adcClk ) ) then
            load     <= genLoad;
            adcCoeff <= resize( signed( toSlv( regs( 0 to  2) ) ), adcCoeff'length );
            adcCini  <= resize( signed( toSlv( regs( 3 to  5) ) ), adcCini'length  );
            adcACini <= resize( signed( toSlv( regs( 6 to 11) ) ), adcACini'length );
            adcASini <= resize( signed( toSlv( regs(12 to 17) ) ), adcASini'length );
            adcAini  <= signed( regs(6) );
         end if;
      end process P_REG_PARMS;

      adcDatA <= std_logic_vector( adcCos(adcCos'left downto adcCos'left - ADC_BITS_G + 1 ) ) & '0';
      adcDatB <= std_logic_vector( adcSin(adcSin'left downto adcSin'left - ADC_BITS_G + 1 ) ) & '0';
   end generate G_ADC_SIN;

   bbi           <= bbo;

end architecture rtl;
