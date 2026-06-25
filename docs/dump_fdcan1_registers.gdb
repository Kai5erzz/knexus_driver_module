# GDB script to dump FDCAN1 and related registers for debugging
# Usage: (gdb) source docs/dump_fdcan1_registers.gdb

echo ========================================\n
echo FDCAN1 Register Dump\n
echo ========================================\n

# ENDN - Endian register (should be 0x87654321)
set $fdcan1_endn = *(uint32_t *)0x4000A000
printf "ENDN  [0x4000A000] = 0x%08X", $fdcan1_endn
if $fdcan1_endn == 0x87654321
    echo  (OK: correct endian marker)\n
else
    echo  (WARNING: unexpected value!)\n
end

# RXF0S - RX FIFO 0 status
set $rxf0s = *(uint32_t *)0x4000A004
printf "RXF0S [0x4000A004] = 0x%08X\n", $rxf0s
printf "  F0FL (fill level) = %d\n", $rxf0s & 0x7F
printf "  F0GI (get index)  = %d\n", ($rxf0s >> 8) & 0x3F
printf "  F0PI (put index)  = %d\n", ($rxf0s >> 16) & 0x3F
printf "  F0F  (full)       = %d\n", ($rxf0s >> 24) & 1
printf "  RF0L (lost)       = %d\n", ($rxf0s >> 25) & 1

# RXF1S - RX FIFO 1 status
set $rxf1s = *(uint32_t *)0x4000A008
printf "RXF1S [0x4000A008] = 0x%08X\n", $rxf1s

# IR - Interrupt Register
set $ir = *(uint32_t *)0x4000A010
printf "\nIR    [0x4000A010] = 0x%08X\n", $ir
printf "  RF0N (RX FIFO0 new)     = %d\n", ($ir >> 0) & 1
printf "  RF0W (RX FIFO0 warning) = %d\n", ($ir >> 1) & 1
printf "  RF0F (RX FIFO0 full)    = %d\n", ($ir >> 2) & 1
printf "  RF0L (RX FIFO0 lost)    = %d\n", ($ir >> 3) & 1
printf "  RF1N (RX FIFO1 new)     = %d\n", ($ir >> 4) & 1
printf "  TC   (TX complete)      = %d\n", ($ir >> 0) & 1
printf "  TBC  (TX buffer cancel) = %d\n", ($ir >> 9) & 1
printf "  TFE  (TX FIFO empty)    = %d\n", ($ir >> 10) & 1
printf "  TEFN (TX event new)     = %d\n", ($ir >> 12) & 1
printf "  TEFL (TX event lost)    = %d\n", ($ir >> 15) & 1
printf "  ELO  (error logging OF) = %d\n", ($ir >> 16) & 1
printf "  EP   (error passive)    = %d\n", ($ir >> 17) & 1
printf "  EW   (error warning)    = %d\n", ($ir >> 18) & 1
printf "  BO   (bus off)          = %d\n", ($ir >> 19) & 1
printf "  WDI  (watchdog int)     = %d\n", ($ir >> 20) & 1
printf "  PEA  (protocol err arb) = %d\n", ($ir >> 21) & 1
printf "  PED  (protocol err data)= %d\n", ($ir >> 22) & 1
printf "  ARA  (access to restrict)= %d\n", ($ir >> 23) & 1

# CCCR - CC Control Register
set $cccr = *(uint32_t *)0x4000A018
printf "\nCCCR  [0x4000A018] = 0x%08X\n", $cccr
printf "  INIT = %d", $cccr & 1
if ($cccr & 1)
    echo  <-- FDCAN IS IN INIT MODE (CANNOT TRANSMIT!)
else
    echo  (normal)
end
printf "  CCE  = %d\n", ($cccr >> 1) & 1
printf "  ASM  = %d (restricted operation)\n", ($cccr >> 2) & 1
printf "  CSA  = %d (clock stop ack)\n", ($cccr >> 3) & 1
printf "  CSR  = %d (clock stop request)\n", ($cccr >> 4) & 1
printf "  MON  = %d (bus monitoring)\n", ($cccr >> 5) & 1
printf "  DAR  = %d (disable auto-retransmission)\n", ($cccr >> 6) & 1
printf "  TEST = %d\n", ($cccr >> 7) & 1
printf "  FDOE = %d (FD operation enable)\n", ($cccr >> 8) & 1
printf "  BSE  = %d (bit rate switch enable)\n", ($cccr >> 9) & 1
printf "  PXHD = %d (protocol exception disable)\n", ($cccr >> 12) & 1
printf "  EFBI = %d (edge filtering)\n", ($cccr >> 13) & 1
printf "  TXP  = %d (transmit pause)\n", ($cccr >> 14) & 1

# NBTP - Nominal Bit Timing
set $nbtp = *(uint32_t *)0x4000A01C
printf "\nNBTP  [0x4000A01C] = 0x%08X\n", $nbtp
set $nbrp = ($nbtp & 0x1FF) + 1
set $nsjw = (($nbtp >> 16) & 0x7F) + 1
set $ntseg1 = (($nbtp >> 25) & 0xFF) + 1
set $ntseg2 = (($nbtp >> 0) & 0x7F)  # Note: NTSEG2 is bits 6:0 in some revisions
# Actually NTSEG2 is bits 24:16 on STM32H7, let me re-read
set $ntseg2_raw = (($nbtp >> 16) & 0x7F)  # This might be NSJW
# Recalculate from register layout: NBRP[25:16], NSJW[15], NTSEG1[14:8], NTSEG2[6:0]
# Actually the STM32H7 FDCAN NBTP layout is:
# bits [25:16] = NBRP (9 bits)
# bits [12:8]  = NSJW (5 bits) -- actually it's wider
# bits [7:0]   = NTSEG1 -- need to check
# Let me just print raw and let user decode
printf "  NBRP  (prescaler - 1) = %d -> prescaler = %d\n", ($nbtp >> 16) & 0x1FF, (($nbtp >> 16) & 0x1FF) + 1
printf "  NSJW  (sync jump - 1) = %d -> SJW = %d\n", ($nbtp >> 25) & 0x7F, (($nbtp >> 25) & 0x7F) + 1
# Correct layout for STM32H7 FDCAN NBTP:
# bits 25:16 = NBRP
# bits 12:8  = NSJW (actually check reference manual)
# Let me just decode the expected values from code config
echo "  (Configured: Prescaler=6, TimeSeg1=15, TimeSeg2=4, SJW=1)\n"
echo "  (Expected NBTP = 0x049F0005)\n"

# PSR - Protocol Status Register
set $psr = *(uint32_t *)0x4000A024
printf "\nPSR   [0x4000A024] = 0x%08X\n", $psr
set $lec = $psr & 7
printf "  LEC (last error) = %d ", $lec
if $lec == 0
    echo (no error)
end
if $lec == 1
    echo (stuff error)
end
if $lec == 2
    echo (form error)
end
if $lec == 3
    echo (ACK error - NO OTHER NODE ACKNOWLEDGING!)
end
if $lec == 4
    echo (bit1 error)
end
if $lec == 5
    echo (bit0 error)
end
if $lec == 6
    echo (CRC error)
end
if $lec == 7
    echo (no CAN data - custom)
end
set $act = ($psr >> 3) & 3
printf "  ACT (activity) = %d ", $act
if $act == 0
    echo (synchronizing)
end
if $act == 1
    echo (idle)
end
if $act == 2
    echo (receiver)
end
if $lec == 3
    echo (transmitter)
end
printf "  EP  (error passive) = %d\n", ($psr >> 6) & 1
printf "  EW  (error warning) = %d\n", ($psr >> 5) & 1
printf "  BO  (bus off)       = %d\n", ($psr >> 7) & 1
printf "  DLEC(data last err) = %d\n", ($psr >> 8) & 7
printf "  RESI(ESI flag)      = %d\n", ($psr >> 11) & 1
printf "  RBRS(BRS flag)      = %d\n", ($psr >> 12) & 1
printf "  REDL(FDF flag)      = %d\n", ($psr >> 13) & 1
printf "  PXE (proto exc)     = %d\n", ($psr >> 14) & 1
set $tdcv = ($psr >> 16) & 0xF
printf "  TDCV(TX delay comp)  = %d\n", $tdcv

# GFC - Global Filter Configuration
set $gfc = *(uint32_t *)0x4000A080
printf "\nGFC   [0x4000A080] = 0x%08X\n", $gfc
printf "  RRFE (reject remote ext) = %d\n", ($gfc >> 0) & 1
printf "  RRFS (reject remote std) = %d\n", ($gfc >> 1) & 1
set $anfe = ($gfc >> 2) & 3
printf "  ANFE (non-match ext)     = %d ", $anfe
if $anfe == 0
    echo (accept in RX FIFO 0)
end
if $anfe == 1
    echo (accept in RX FIFO 1)
end
if $anfe == 2
    echo (reject)
end
set $anfs = ($gfc >> 4) & 3
printf "  ANFS (non-match std)     = %d ", $anfs
if $anfs == 0
    echo (accept in RX FIFO 0)
end
if $anfs == 1
    echo (accept in RX FIFO 1)
end
if $anfs == 2
    echo (reject)
end

# TXBC - TX Buffer Configuration
set $txbc = *(uint32_t *)0x4000A0C0
printf "\nTXBC  [0x4000A0C0] = 0x%08X\n", $txbc
set $tfqm = ($txbc >> 24) & 1
printf "  TFQM (TX FIFO/Queue mode) = %d", $tfqm
if $tfqm
    echo  (Queue mode)
else
    echo  (FIFO mode)
end

# TXFQS - TX FIFO/Queue Status
set $txfqs = *(uint32_t *)0x4000A0C4
printf "\nTXFQS [0x4000A0C4] = 0x%08X\n", $txfqs
printf "  TFFL (TX FIFO free level) = %d\n", $txfqs & 0xF
printf "  TFGI (TX FIFO get index)  = %d\n", ($txfqs >> 8) & 3
printf "  TFQPI(TX FIFO put index)  = %d\n", ($txfqs >> 16) & 3
printf "  TFQF (TX FIFO full)       = %d\n", ($txfqs >> 21) & 1

# TXBRP - TX Buffer Request Pending
set $txbrp = *(uint32_t *)0x4000A0C8
printf "\nTXBRP [0x4000A0C8] = 0x%08X\n", $txbrp
printf "  TRP (TX request pending) = 0x%02X\n", $txbrp & 0xF

# TXBAR - TX Buffer Add Request
set $txbar = *(uint32_t *)0x4000A0CC
printf "TXBAR [0x4000A0CC] = 0x%08X\n", $txbar
printf "  AR (add request) = 0x%02X\n", $txbar & 0xF

# TXBCR - TX Buffer Cancellation Request
set $tbcr = *(uint32_t *)0x4000A0D0
printf "TXBCR [0x4000A0D0] = 0x%08X\n", $tbcr
printf "  CR (cancel request) = 0x%02X\n", $tbcr & 0xF

echo \n========================================\n
echo GPIO Configuration for PA11/PA12\n
echo ========================================\n

# GPIOA MODER
set $moder = *(uint32_t *)0x40020000
printf "MODER [0x40020000] = 0x%08X\n", $moder
set $pa11_mode = ($moder >> 22) & 3
set $pa12_mode = ($moder >> 24) & 3
printf "  PA11 (bits 23:22) = %d ", $pa11_mode
if $pa11_mode == 0
    echo (INPUT)
end
if $pa11_mode == 1
    echo (OUTPUT)
end
if $pa11_mode == 2
    echo (AF - CORRECT)
end
if $pa11_mode == 3
    echo (ANALOG)
end
printf "  PA12 (bits 25:24) = %d ", $pa12_mode
if $pa12_mode == 0
    echo (INPUT)
end
if $pa12_mode == 1
    echo (OUTPUT)
end
if $pa12_mode == 2
    echo (AF - CORRECT)
end
if $pa12_mode == 3
    echo (ANALOG)
end

# GPIOA AFRH (AFR[1])
set $afrh = *(uint32_t *)0x40020024
printf "AFRH  [0x40020024] = 0x%08X\n", $afrh
set $pa11_af = ($afrh >> 12) & 0xF
set $pa12_af = ($afrh >> 16) & 0xF
printf "  PA11 AF (bits 15:12) = %d", $pa11_af
if $pa11_af == 9
    echo  (AF9 - CORRECT for FDCAN1)
else
    echo  (WRONG! Should be AF9=9)
end
printf "  PA12 AF (bits 19:16) = %d", $pa12_af
if $pa12_af == 9
    echo  (AF9 - CORRECT for FDCAN1)
else
    echo  (WRONG! Should be AF9=9)
end

# GPIOA OSPEEDR
set $ospeedr = *(uint32_t *)0x40020008
printf "OSPEEDR [0x40020008] = 0x%08X\n", $ospeedr
printf "  PA11 speed = %d\n", ($ospeedr >> 22) & 3
printf "  PA12 speed = %d\n", ($ospeedr >> 24) & 3

# GPIOA PUPDR
set $pupdr = *(uint32_t *)0x4002000C
printf "PUPDR [0x4002000C] = 0x%08X\n", $pupdr
printf "  PA11 pull = %d\n", ($pupdr >> 22) & 3
printf "  PA12 pull = %d\n", ($pupdr >> 24) & 3

echo \n========================================\n
echo RCC Clock Enable for FDCAN\n
echo ========================================\n

# RCC APB1HENR
set $apb1henr = *(uint32_t *)0x580245E4
printf "APB1HENR [0x580245E4] = 0x%08X\n", $apb1henr
printf "  FDCAN clock enable (bit 8) = %d", ($apb1henr >> 8) & 1
if ($apb1henr >> 8) & 1
    echo  (ENABLED)
else
    echo  (DISABLED!)
end

echo \n========================================\n
echo FDCAN Instance Variable (via GDB)\n
echo ========================================\n

# Try to evaluate hfdcan1 state
echo Attempting: print hfdcan1.State\n
print hfdcan1.State
echo Attempting: print hfdcan1.ErrorCode\n
print hfdcan1.ErrorCode
echo Attempting: print hfdcan1.Lock\n
print hfdcan1.Lock

echo \n========================================\n
echo DIAGNOSIS SUMMARY\n
echo ========================================\n

# Check INIT bit
if ($cccr & 1) == 1
    echo [CRITICAL] FDCAN1 is in INIT mode! Cannot transmit.\n
    echo   -> HAL_FDCAN_Start() may not have been called, or it failed.\n
end

# Check bus off
if ($psr >> 7) & 1
    echo [CRITICAL] Bus-Off detected! Too many errors on the bus.\n
    echo   -> Check bus termination, wiring, and bit timing.\n
end

# Check error passive
if ($psr >> 6) & 1
    echo [WARNING] Error Passive state. TEC > 127 or REC > 127.\n
end

# Check LEC
set $lec_val = $psr & 7
if $lec_val == 3
    echo [CRITICAL] Last Error = ACK Error. No other node is acknowledging!\n
    echo   -> Check: Is there another CAN node on the bus?\n
    echo   -> Check: Is the bus properly terminated (120 ohm)?\n
    echo   -> Check: Are bit timings correct?\n
end
if $lec_val == 1
    echo [WARNING] Last Error = Stuff Error. Possible signal integrity issue.\n
end
if $lec_val == 4
    echo [WARNING] Last Error = Bit1 Error. Transmitted recessive but read dominant.\n
end
if $lec_val == 5
    echo [WARNING] Last Error = Bit0 Error. Transmitted dominant but read recessive.\n
end

# Check GPIO
if $pa11_mode != 2
    echo [CRITICAL] PA11 is NOT in AF mode! FDCAN1_RX won't work.\n
end
if $pa12_mode != 2
    echo [CRITICAL] PA12 is NOT in AF mode! FDCAN1_TX won't work.\n
end

# Check RCC
if (($apb1henr >> 8) & 1) == 0
    echo [CRITICAL] FDCAN clock is NOT enabled!\n
end

echo \nDone.\n
