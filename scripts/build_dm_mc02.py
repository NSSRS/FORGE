#!/usr/bin/env python3
"""Build MC02 USB CDC + three classic-CAN buses. Never flashes hardware."""
import argparse
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'build/dm_mc02'
SDK = BUILD / 'sdk'
FW = ROOT / 'src/damiao_firmware'

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cc', default='arm-none-eabi-gcc')
    args = parser.parse_args()
    cc = shutil.which(args.cc)
    if not cc:
        raise SystemExit('Install gcc-arm-none-eabi, binutils-arm-none-eabi and libnewlib-arm-none-eabi, or pass --cc.')
    if not (SDK / 'USB_DEVICE/Target/usbd_conf.c').exists():
        raise SystemExit('Run python3 scripts/fetch_mc02_sdk.py first')
    generated = BUILD / 'generated'
    generated.mkdir(parents=True, exist_ok=True)
    # Adapt build copies only. Cached vendor sources stay byte-identical.
    main_c = (SDK / 'Core/Src/main.c').read_text()
    main_c = main_c.replace('#include "vofa.h"', '#include "board.h"')
    main_c = main_c.replace('MX_USB_DEVICE_Init();', 'forge_init();\n  MX_USB_DEVICE_Init();')
    main_c = main_c.replace('vofa_start();', 'forge_poll();')
    (generated / 'main.c').write_text(main_c)
    cdc = (SDK / 'USB_DEVICE/App/usbd_cdc_if.c').read_text()
    cdc = '#include "board.h"\n' + cdc
    cdc = cdc.replace('/* USER CODE BEGIN 11 */', '/* USER CODE BEGIN 11 */\n  forge_usb_receive(Buf, *Len);')
    cdc = cdc.replace('if (hcdc->TxState != 0)', 'if (hcdc == NULL || hcdc->TxState != 0)')
    (generated / 'usbd_cdc_if.c').write_text(cdc)
    includes = [FW/'include', FW/'board', SDK/'Core/Inc', SDK/'Drivers/STM32H7xx_HAL_Driver/Inc',
                SDK/'Drivers/CMSIS/Include', SDK/'Drivers/CMSIS/Device/ST/STM32H7xx/Include',
                SDK/'USB_DEVICE/App', SDK/'USB_DEVICE/Target',
                SDK/'Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
                SDK/'Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc']
    sources = list((FW/'src').glob('*.c')) + list((FW/'board').glob('*.c'))
    sources += [p for p in (SDK/'Core/Src').glob('*.c') if p.name!='main.c']
    sources += list((SDK/'Drivers/STM32H7xx_HAL_Driver/Src').glob('*.c'))
    sources += [p for p in (SDK/'USB_DEVICE').rglob('*.c') if p.name!='usbd_cdc_if.c']
    sources += list((SDK/'Middlewares').rglob('*.c')) + list(generated.glob('*.c'))
    flags = ['-mcpu=cortex-m7','-mthumb','-mfpu=fpv5-d16','-mfloat-abi=hard','-Os','-g3',
             '-ffunction-sections','-fdata-sections','-std=gnu11','-Wall','-Wextra',
             '-DSTM32H723xx','-DUSE_HAL_DRIVER','-DHAL_FDCAN_MODULE_ENABLED']
    # Ubuntu's relocated package can be used without system installation.
    prefix = Path(cc).resolve().parents[1]
    newlib = prefix/'lib/arm-none-eabi/newlib'
    if newlib.exists():
        flags += ['-B'+str(newlib)+'/', '-I'+str(prefix/'include/newlib')]
    inc = ['-I'+str(p) for p in includes]
    objects=[]
    for index, source in enumerate(sources):
        obj=BUILD/f'{index}_{source.stem}.o'; objects.append(obj)
        subprocess.run([cc,*flags,*inc,'-c',str(source),'-o',str(obj)],check=True)
    elf=BUILD/'forge_arm.elf'
    # Select the matching multilib explicitly when using a relocated Debian SDK.
    libs=[]
    if newlib.exists(): libs=['-L'+str(newlib/'thumb/v7e-m+dp/hard')]
    subprocess.run([cc,*flags,*libs,*map(str,objects),'-T'+str(FW/'board/flash.ld'),
        '-Wl,--gc-sections,-Map='+str(BUILD/'forge_arm.map'),'--specs=nano.specs','--specs=nosys.specs',
        '-nostartfiles','-Wl,--start-group','-lc','-lm','-lnosys','-Wl,--end-group','-o',str(elf)],check=True)
    objcopy=cc.replace('gcc','objcopy'); size=cc.replace('gcc','size')
    subprocess.run([objcopy,'-O','binary',str(elf),str(BUILD/'forge_arm.bin')],check=True)
    subprocess.run([size,str(elf)],check=True)
    print(f'Built {elf}; no hardware was flashed.')
if __name__=='__main__': main()
