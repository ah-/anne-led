target remote :3333
set print asm-demangle on
monitor arm semihosting enable
#b main
#load
#b anne_led::main
#c
