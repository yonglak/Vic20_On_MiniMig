@echo off
set rom_path=..\ROM\ROM\
set dest_rom_path=..\

romgen %rom_path%basic.bin VIC20_BASIC_ROM 13 l r e> %dest_rom_path%vic20_basic.vhd
romgen %rom_path%kernal.bin VIC20_KERNAL_ROM 13 l r e> %dest_rom_path%vic20_kernal.vhd
romgen %rom_path%characters.bin VIC20_CHAR_ROM 12 l r e > %dest_rom_path%vic20_chars.vhd

echo done
