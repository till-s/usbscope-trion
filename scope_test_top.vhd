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
use     work.BasicPkg.NaturalArray;
use     work.AcqCtlPkg.all;
use     work.SDRAMPkg.all;
use     work.GitVersionPkg.all;

entity scope_test_top is
   generic (
      USE_SMPL_CLK_G    : boolean := false;
      -- pipeline stages in the readback:
      --   external registers (in IO ring) (1 in out, 1 in in direction)
      --   sampling and resync to sdram_clk; the precise value also depends
      --   on the fine-tuning of the sdram_smpl_clk phase...
      SDRAM_READ_DLY_G  : natural := 3;
      BOARD_VERSION_G   : std_logic_vector( 7 downto 0) := x"02";
      USE_SDRAM_BUF_G   : boolean := true;
      SDRAM_LD_ROWS_G   : natural := 13; -- log2 of number of rows
      SDRAM_LD_COLS_G   : natural := 9;  -- log2 of number of columns
      SDRAM_LD_BNKS_G   : natural := 2;  -- log2 of number of banks
      -- block-ram depth (# samples) if USE_SDRAM_BUF_G is false; ignored otherwise
      BRAM_LD_DEPTH_G   : natural := 11;
      ADC_FREQ_G        : real    := 130.0E6;
      RAM_FREQ_G        : real    := 166.0E6;
      NO_DECIMATORS_G   : boolean := false;
      HAVE_SPI_CMD_G    : boolean := true;
      ADC_BITS_G        : natural := 10;
      USE_STRM_BUF_G    : boolean := true
   );
   port (
      sdram_clk         : in  std_logic;
      sdram_smpl_clk    : in  std_logic;
      -- create output clock
      sdram_clkout_HI   : out std_logic := '1';
      sdram_clkout_LO   : out std_logic := '0';

      sdram_CSb         : out std_logic := '1';
      sdram_CKE         : out std_logic := '0';
      sdram_DQML        : out std_logic := '0';
      sdram_DQMH        : out std_logic := '0';
      sdram_WEb         : out std_logic := '1';
      sdram_CASb        : out std_logic := '1';
      sdram_RASb        : out std_logic := '1';
      sdram_A           : out std_logic_vector(12 downto 0) := (others => '0');
      sdram_BA          : out std_logic_vector( 1 downto 0) := (others => '0');
      sdram_DQ_IN       : in  std_logic_vector(15 downto 0) := (others => '0');
      sdram_DQ_OUT      : out std_logic_vector(15 downto 0) := (others => '0');
      sdram_DQ_OE       : out std_logic_vector(15 downto 0) := (others => '0');
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

      led               : out   std_logic_vector(12 downto 0) := (others => '0');

      i2cSDA_IN         : in    std_logic;
      i2cSDA_OUT        : out   std_logic := '0';
      i2cSDA_OE         : out   std_logic := '0';
      i2cSCL_IN         : in    std_logic;
      i2cSCL_OUT        : out   std_logic := '0';
      i2cSCL_OE         : out   std_logic := '0';

      gpioDat_IN        : in    std_logic;
      gpioDat_OUT       : out   std_logic := '0';
      gpioDat_OE        : out   std_logic := '0';
      gpioDir           : out   std_logic := '0';

      adcSClk           : out   std_logic := '0';
      adcSDIO_IN        : in    std_logic;
      adcSDIO_OUT       : out   std_logic := '0';
      adcSDIO_OE        : out   std_logic := '0';
      adcCSb            : out   std_logic := '1';

      pgaSClk           : out   std_logic := '0';
      pgaSDat           : out   std_logic := '0';
      pgaCSb            : out   std_logic_vector(1 downto 0) := (others => '1');

      spiSClk           : out   std_logic;
      spiMOSI           : out   std_logic;
      spiMISO           : in    std_logic;
      spiCSb            : out   std_logic;

      extIO             : in    std_logic_vector(1 downto 0);

      adcClk            : in    std_logic;
      adcPllLocked      : in    std_logic;
      ADC_DDR_HI        : in    std_logic_vector(ADC_BITS_G downto 0);
      ADC_DDR_LO        : in    std_logic_vector(ADC_BITS_G downto 0);

      -- aux/fallback clock
      fpgaClk           : in    std_logic
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

   function SDRAM_NSMPL_MAX_F(constant x : natural; constant align : natural) return natural is
      variable v : natural;
   begin
      v := x;
      if ( USE_SDRAM_BUF_G ) then
         v := v * 8 / ADC_BITS_G;
      end if;
      v := v / align;
      v := v * align;
      return v;
   end function SDRAM_NSMPL_MAX_F;

   -- must cover bulk max pkt size
   constant LD_FIFO_OUT_C      : natural :=  9;
   constant LD_FIFO_INP_C      : natural :=  9;
   constant SDRAM_A_WIDTH_C    : natural := SDRAM_LD_ROWS_G;
   constant SDRAM_B_WIDTH_C    : natural := SDRAM_LD_BNKS_G;
   constant SDRAM_C_WIDTH_C    : natural := SDRAM_LD_COLS_G;
   constant FLAT_A_WIDTH_C     : natural := SDRAM_A_WIDTH_C + SDRAM_B_WIDTH_C + SDRAM_C_WIDTH_C;
   -- make a multiple of 1024
   constant SDRAM_NSMPL_MAX_C  : natural := SDRAM_NSMPL_MAX_F(2**FLAT_A_WIDTH_C, 1024);

   constant ULPI_CLK_FREQ_C    : real    := 60.0E6;
   constant ACM_CLK_FREQ_C     : real    := ULPI_CLK_FREQ_C;

   constant MEM_DEPTH_C        : natural := ite( USE_SDRAM_BUF_G, SDRAM_NSMPL_MAX_C, 2**BRAM_LD_DEPTH_G );

   function BB_DELAY_ARRAY_F   return NaturalArray is
      variable v : NaturalArray( 0 to 2**SubCommandBBType'length - 1 ) := (others => 1);
   begin
      v( to_integer( unsigned( CMD_BB_NONE_C    ) ) ) := 0;
      -- 10k pulldowns make the PGA level shifters really slow; measured time
      -- constant for HI-LO transition (at shifter output) ~330ns!
      v( to_integer( unsigned( CMD_BB_SPI_PGA_C ) ) ) := 200;
      return v;
   end function BB_DELAY_ARRAY_F;

   constant BB_DELAY_ARRAY_C   : NaturalArray := BB_DELAY_ARRAY_F;

   constant BB_SPI_CSb_C       : natural := 0;
   constant BB_SPI_SCK_C       : natural := 1;
   constant BB_SPI_MSO_C       : natural := 2;
   constant BB_SPI_MSI_C       : natural := 3;

   constant BB_I2C_SDA_C       : natural := 4;
   constant BB_I2C_SCL_C       : natural := 5;

   constant BB_SPI_T_C         : natural := 6;
   constant BB_XXX_XXX_C       : natural := 7;

   constant BB_INIT_C          : std_logic_vector(7 downto 0) := x"F1";

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

   signal sdramDQOE            : std_logic    := '0';

   signal ulpiIb               : UlpiIbType   := ULPI_IB_INIT_C;
   signal ulpiOb               : UlpiObType   := ULPI_OB_INIT_C;
   signal ulpiRx               : UlpiRxType;
   signal ulpiRst              : std_logic    := '0';
   signal ulpiForceStp         : std_logic    := '0';
   signal usb2HiSpeedEn        : std_logic    := '1';
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
   signal adcStatus            : std_logic_vector(7 downto 0);

   signal regRDat              : std_logic_vector(7 downto 0) := (others => '0');
   signal regWDat              : std_logic_vector(7 downto 0);
   signal regAddr              : unsigned(7 downto 0);
   signal regRdnw              : std_logic;
   signal regVld               : std_logic;
   signal regRdy               : std_logic := '1';
   signal regErr               : std_logic := '1';

   type RegType is record
      led         : std_logic_vector(led'range);
      isTriggered : std_logic;
      sel         : unsigned(7 downto 0);
   end record RegType;

   constant REG_INIT_C : RegType := (
      led         => (others => '0'),
      isTriggered => '0',
      sel         => (others => '1')
   );

   signal regs                 : RegType   := REG_INIT_C;
   signal regsIn               : RegType;
   signal isTriggeredLoc       : std_logic := '0';

   signal spiSClkCtl           : std_logic;
   signal spiMOSICtl           : std_logic;
   signal spiCSbCtl            : std_logic;

   signal extTrg               : std_logic := '0';
   signal extTrgOut            : std_logic := '0';
   signal extTrgOutEn          : std_logic := '0';
   signal extTrgOutEnLst       : std_logic := '0';
   signal gpioIsOutput         : std_logic;

   signal pgaCSbLocIb          : std_logic;
   signal pgaCSbLocOb          : std_logic_vector(pgaCSb'range) := (others => '1');
   signal pgaMOSILoc           : std_logic;
   signal pgaMISOLoc           : std_logic;
   signal pgaSClkLocIb         : std_logic;
   signal pgaSClkLocOb         : std_logic_vector(pgaCSb'range);

begin

   assert not USE_SDRAM_BUF_G or ADC_BITS_G = 10 or ADC_BITS_G = 8
      report "SDRAM buffer only supports 10 or 8 ADC bits"
      severity failure;

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

   G_STRM_BUF : if ( USE_STRM_BUF_G ) generate

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

   end generate G_STRM_BUF;

   G_NO_STRM_BUF : if ( not USE_STRM_BUF_G ) generate
      fifoRVld      <= acmFifoOutVld;
      fifoRDat      <= acmFifoOutDat;
      acmFifoOutRen <= fifoRRdy;
   end generate G_NO_STRM_BUF;

   acmFifoInpDat <= fifoWDat;
   acmFifoInpWen <= fifoWVld;
   fifoWRdy      <= not acmFifoInpFull;

   U_CMD : entity work.CommandWrapper
   generic map (
      I2C_SCL_G                    => BB_I2C_SCL_C,
      BBO_INIT_G                   => BB_INIT_C,
      FIFO_FREQ_G                  => 60.0E6,
      ADC_FREQ_G                   => ADC_FREQ_G,
      ADC_BITS_G                   => ADC_BITS_G,
      RAM_BITS_G                   => ADC_BITS_G,
      MEM_DEPTH_G                  => MEM_DEPTH_C,
      GIT_VERSION_G                => GIT_VERSION_C,
      BOARD_VERSION_G              => BOARD_VERSION_G,
      BB_DELAY_ARRAY_G             => BB_DELAY_ARRAY_C,
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

      adcStatus                    => adcStatus, --: out std_logic_vector(7 downto 0) := (others => '0');
      err(0)                       => open,
      err(1)                       => open,

      -- register interface
      regClk                       => ulpiClk,
      regRDat                      => regRDat, --: in  std_logic_vector(7 downto 0) := (others => '0');
      regWDat                      => regWDat, --: out std_logic_vector(7 downto 0);
      regAddr                      => regAddr, --: out unsigned(7 downto 0);
      regRdnw                      => regRdnw, --: out std_logic;
      regVld                       => regVld,  --: out std_logic;
      regRdy                       => regRdy,  --: in  std_logic := '1';
      regErr                       => regErr,  --: in  std_logic := '1'

      spiSClk                      => spiSClkCtl, --: out std_logic;
      spiMOSI                      => spiMOSICtl, --: out std_logic;
      spiCSb                       => spiCSbCtl,  --: out std_logic;
      spiMISO                      => spiMISO, --: in  std_logic := '0';

      adcClk                       => adcClk,
      adcRst                       => open,

      -- bit 0 is the DOR (overrange) bit
      adcDataA                     => adcDatAReg,
      adcDataB                     => adcDatBReg,

      extTrg                       => extTrg,
      extTrgOut                    => extTrgOut,
      extTrgOutEn                  => extTrgOutEn,

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
            CLK_FREQ_G                => RAM_FREQ_G,
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

   G_SDRAM_DBG : if ( not USE_SDRAM_BUF_G ) generate

      type SDRAMDbgType is record
         CSb         : std_logic;
         CKE         : std_logic;
         DQML        : std_logic;
         DQMH        : std_logic;
         WEb         : std_logic;
         A           : std_logic_vector(SDRAM_A_WIDTH_C - 1 downto 0);
         CASb        : std_logic;
         RASb        : std_logic;
         BA          : std_logic_vector( 1 downto 0);
         DQ_OUT      : std_logic_vector(15 downto 0);
         DQOE        : std_logic;
      end record SDRAMDbgType;

      constant SDRAM_DBG_INIT_C : SDRAMDbgType := (
         CSb         => '1',
         CKE         => '0',
         DQML        => '0',
         DQMH        => '0',
         WEb         => '1',
         A           => (others => '0'),
         CASb        => '1',
         RASb        => '1',
         BA          => (others => '0'),
         DQ_OUT      => (others => '0'),
         DQOE        => '0'
      );

      signal r   : SDRAMDbgType := SDRAM_DBG_INIT_C;

   begin
      P_CMB : process ( regs, r ) is
         variable v : SDRAMDbgType;
         variable s : natural;
      begin
         v := SDRAM_DBG_INIT_C;
         s := to_integer(regs.sel);
         if    ( s < r.DQ_OUT'length                                 ) then
            v.DQOE        := '1';
            v.DQ_OUT( s ) := '1';
         elsif ( s < r.DQ_OUT'length + r.A'length                    ) then
            s := s - r.DQ_OUT'length;
            v.A( s )      := '1';
         elsif ( s < r.DQ_OUT'length + r.A'length + r.BA'length      ) then
            s := s - r.DQ_OUT'length - r.A'length;
            v.BA(s)       := '1';
         elsif ( s < r.DQ_OUT'length + r.A'length + r.BA'length + 1  ) then
            v.CSb         := '0';
         elsif ( s < r.DQ_OUT'length + r.A'length + r.BA'length + 2  ) then
            v.CKE         := '1';
         elsif ( s < r.DQ_OUT'length + r.A'length + r.BA'length + 3  ) then
            v.DQML        := '1';
         elsif ( s < r.DQ_OUT'length + r.A'length + r.BA'length + 4  ) then
            v.DQMH        := '1';
         elsif ( s < r.DQ_OUT'length + r.A'length + r.BA'length + 5  ) then
            v.WEb         := '0';
         elsif ( s < r.DQ_OUT'length + r.A'length + r.BA'length + 6  ) then
            v.CASb        := '0';
         elsif ( s < r.DQ_OUT'length + r.A'length + r.BA'length + 7  ) then
            v.RASb        := '0';
         end if;
         r <= v;
      end process P_CMB;

      sdram_CSb              <= r.CSb;
      sdram_CKE              <= r.CKE;
      sdram_DQML             <= r.DQML;
      sdram_DQMH             <= r.DQMH;
      sdram_A(r.A'range)     <= r.A;
      sdram_WEb              <= r.WEb;
      sdram_CASb             <= r.CASb;
      sdram_RASb             <= r.Rasb;
      sdram_BA               <= r.BA;
      sdram_DQ_OUT           <= r.DQ_OUT;
      sdramDQOE              <= r.DQOE;
   end generate G_SDRAM_DBG;

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

   B_IO : block is
   begin

      extTrg        <= gpioDat_IN;
      gpioDat_OUT   <= extTrgOut;
      gpioDat_OE    <= (extTrgOutEn and extTrgOutEnLst);
      gpioIsOutput  <= (extTrgOutEn or  extTrgOutEnLst);
      gpioDir       <= gpioIsOutput;

      i2cSDA_OUT    <= bbo(BB_I2C_SDA_C);
      i2cSDA_OE     <= not bbo(BB_I2C_SDA_C);
      bbi(BB_I2C_SDA_C) <= i2cSDA_IN;

      i2cSCL_OUT    <= bbo(BB_I2C_SCL_C);
      i2cSCL_OE     <= not bbo(BB_I2C_SCL_C);
      bbi(BB_I2C_SCL_C) <= i2cSCL_IN;

      -- write to device only if T is deasserted
      pgaCSb(0)         <= not pgaCSBLocOb(0);  -- drivers invert
      pgaCSb(1)         <= not pgaCSBLocOb(1);  -- drivers invert
      -- the pgaSClkLocOb signals are gated; the muxed chip-selects (pgaCSBLocOb)
      -- are asserted early (while a preceding SCLK on the input of the shadow registers may
      -- still be active).
      -- Since we have only a single physical line we or the two gated clocks together.
      -- We MUST NOT use the pgaSClkIb (ungated).
      pgaSClk           <= not (pgaSClkLocOb(0) or pgaSClkLocOb(1)); -- drivers invert
      pgaSDat           <= not pgaMOSILoc; -- drivers invert

      P_CS_MUX : process ( bbo, subCmdBB, adcSDIO_IN, spiMISO, pgaMISOLoc, spiSClkCtl, spiMOSICtl, spiCSbCtl ) is
         variable spiCSbLoc : std_logic;
      begin
         adcCSb         <= '1';
         spiCSbLoc      := spiCSbCtl;
         pgaCSbLocIb    <= '1';
         adcSDIO_OE     <= '0';
         adcSDIO_OUT    <= '1';

         pgaSClkLocIb   <= bbo(BB_SPI_SCK_C);
         adcSClk        <= bbo(BB_SPI_SCK_C);
         spiSClk        <= bbo(BB_SPI_SCK_C) or spiSClkCtl;

         pgaMOSILoc     <= bbo(BB_SPI_MSO_C);
         spiMOSI        <= bbo(BB_SPI_MSO_C) or spiMOSICtl;

         bbi(BB_SPI_MSI_C)               <= '0';

         if    ( subCmdBB = CMD_BB_SPI_ADC_C ) then
            adcCSb      <= bbo(BB_SPI_CSb_C);
            if ( bbo(BB_SPI_T_C) = '0' ) then
               adcSDIO_OUT  <= bbo(BB_SPI_MSO_C);
               adcSDIO_OE   <= '1';
            end if;
            bbi(BB_SPI_MSI_C) <= adcSDIO_IN;
         elsif ( subCmdBB = CMD_BB_SPI_PGA_C ) then
            pgaCSbLocIb       <= bbo(BB_SPI_CSb_C);
            bbi(BB_SPI_MSI_C) <= pgaMISOLoc;
         elsif ( subCmdBB = CMD_BB_SPI_ROM_C ) then
            spiCSbLoc         := bbo(BB_SPI_CSb_C) and spiCSbCtl;
            bbi(BB_SPI_MSI_C) <= spiMISO;
         end if;
         spiCSb <= spiCSbLoc;
      end process P_CS_MUX;

   end block B_IO;

   P_SIM : process ( adcClk ) is
   begin
      if ( rising_edge( adcClk ) ) then
         if ( adcCnt = -2 ) then
            adcCnt      <= (others => '0');
         else
            adcCnt      <= adcCnt + 1;
         end if;
         adcDatAReg     <= adcDatA;
         adcDatBReg     <= adcDatB;
         extTrgOutEnLst <= extTrgOutEn;
      end if;
   end process P_SIM;

   adcDatA <= ADC_DDR_LO;
   adcDatB <= ADC_DDR_HI;

   process ( ulpiClk ) is
      variable cnt : unsigned(25 downto 0) := (others => '0');
   begin
      if ( rising_edge( ulpiClk ) ) then
         cnt := cnt + 1;
      end if;
   end process;

   U_PGA_REGS : entity work.SpiShadowReg
      generic map (
         NUM_REGS_G => 2,
         REG_INIT_G => (
            0 => x"00",
            1 => x"00"
         )
      )
      port map (
         clk               => ulpiClk,
         -- resetting this does not reset the actual hardware we are caching
         -- rst               => acmFifoRst,
         sclkIb            => pgaSClkLocIb,
         scsbIb            => pgaCSbLocIb,
         mosiIb            => pgaMOSILoc,
         misoIb            => pgaMISOLoc,

         sclkOb            => pgaSClkLocOb,
         scsbOb            => pgaCSbLocOb
      );

   B_REGS : block is
   begin
      P_COMB : process (regs, regVld, regRdnw, regAddr, regWDat) is
         variable v : RegType;
      begin
         v              := regs;
         regRdy         <= '1';
         regErr         <= '0';
         regRDat        <= (others => '0');
         isTriggeredLoc <= regs.isTriggered;
         if     ( regAddr = 0 ) then
            -- front LEDs
            regRDat <= '0' & regs.led(5 downto 3) & '0' & regs.led(8 downto 6);
            if ( (regVld and not regRdnw) = '1' ) then
               v.led(5 downto 3) := regWDat(6 downto 4);
               v.led(8 downto 6) := regWDat(2 downto 0);
            end if;
         elsif  ( regAddr = 1 ) then
            -- rear  LEDs
            regRDat <= regs.led(12 downto 9) & '0' & regs.led(2 downto 0);
            if ( (regVld and not regRdnw) = '1' ) then
               v.led(12 downto 9) := regWDat(7 downto 4);
               v.led( 2 downto 0) := regWDat(2 downto 0);
            end if;
         elsif  ( regAddr = 2 ) then
            regRDat(0) <= regs.isTriggered;
            if ( (regVld and not regRdnw) = '1' ) then
               v.isTriggered      := regWDat(0);
               -- writing a one when bit is already active causes a flicker
               if ( ( v.isTriggered and regs.isTriggered ) = '1' ) then
                  isTriggeredLoc <= '0';
               end if;
            end if;
         elsif  ( not USE_SDRAM_BUF_G and ( regAddr = 3 ) ) then
            regRDat <= std_logic_vector( regs.sel );
            if ( (regVld and not regRdnw) = '1' ) then
               v.sel := unsigned( regWDat );
            end if;
         else
            regErr <= '1';
         end if;

         regsIn  <= v;
      end process P_COMB;

      P_SEQ : process ( ulpiClk ) is
      begin
         if ( rising_edge( ulpiClk ) ) then
            if ( acmFifoRst = '1' ) then
               regs <= REG_INIT_C;
            else
               regs <= regsIn;
            end if;
         end if;
      end process P_SEQ;
   end block B_REGS;

   B_LEDS : block is
      signal isTriggeredAny, isTriggeredA, isTriggeredB, isTriggeredE : std_logic;
   begin

      U_FLICKER : entity work.Flicker
         generic map (
            CLOCK_FREQ_G => ACM_CLK_FREQ_C
         )
         port map (
            clk          => ulpiClk,
            rst          => acmFifoRst,
            datInp       => isTriggeredLoc,
            datOut       => isTriggeredAny
         );

      isTriggeredA   <= isTriggeredAny and adcStatus(ACQ_STA_SRC_A_C);
      isTriggeredB   <= isTriggeredAny and adcStatus(ACQ_STA_SRC_B_C);
      isTriggeredE   <= isTriggeredAny and not adcStatus(ACQ_STA_SRC_A_C) and not adcStatus(ACQ_STA_SRC_B_C);

      P_MAP_LED : process (
         ulpiPllLocked,
         adcPllLocked,
         adcStatus,
         isTriggeredA,
         isTriggeredB,
         isTriggeredE,
         gpioIsOutput,
         regs
      ) is
         variable v : std_logic_vector(led'range);
      begin
         v     := (others => '0');
         v( 0) := '0';                          -- front-right, Red
         v( 1) := isTriggeredE;                 -- front-right, Green
         v( 2) := gpioIsOutput;                 -- front-right, Blue

         v( 3) := adcStatus(ACQ_STA_OVR_A_C);   -- CHA,         Red
         v( 4) := isTriggeredA;                 -- CHA,         Green
         v( 5) := '0';                          -- CHA,         Blue

         v( 6) := adcStatus(ACQ_STA_OVR_B_C);   -- CHB,         Red
         v( 7) := isTriggeredB;                 -- CHB,         Green
         v( 8) := '0';                          -- CHB,         Blue

         v( 9) := '0';                          -- front-left,  Red
         v(10) := ulpiPllLocked and adcPllLocked; -- front-left,  Green
         v(11) := '0';                          -- front-left,  Blue

         v(12) := '0';                          -- front-left, single

         led   <= v or regs.led;
      end process P_MAP_LED;

   end block B_LEDS;

end architecture rtl;
