mkdir Binary
arm-none-eabi-objcopy -O binary "Debug\FlySight_STM32_MMG_New.elf" "Binary\FlySight_STM32_MMG_New.bin"
arm-none-eabi-size "Debug\FlySight_STM32_MMG_New.elf"

PATH=..\FlySight_Bootloader\Middlewares\ST\STM32_Secure_Engine\Utilities\KeysAndImages\win\prepareimage;%PATH%
prepareimage enc -k "..\FlySight_Bootloader\1_Image_SECoreBin\Binary\OEM_KEY_COMPANY1_key_AES_CBC.bin" -i "..\FlySight_Bootloader\1_Image_SECoreBin\Binary\iv.bin" "Binary\FlySight_STM32_MMG_New.bin" "Binary\FlySight_STM32_MMG_New.sfu"
prepareimage sha256 "Binary\FlySight_STM32_MMG_New.bin" "Binary\FlySight_STM32_MMG_New.sign"
prepareimage pack -m "SFU1" -k "..\FlySight_Bootloader\1_Image_SECoreBin\Binary\ECCKEY1.txt" -r 28 -v "1" -i "..\FlySight_Bootloader\1_Image_SECoreBin\Binary\iv.bin" -f "Binary\FlySight_STM32_MMG_New.sfu" -t "Binary\FlySight_STM32_MMG_New.sign" "Binary\FlySight_STM32_MMG_New.sfb" -o 512
