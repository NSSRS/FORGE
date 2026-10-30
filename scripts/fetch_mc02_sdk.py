#!/usr/bin/env python3
"""Fetch a pinned subset of Damiao's STM32 USB example into ignored build/.
Vendor sources are cached unchanged and verified against Git blob SHA1.
"""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import urllib.request

REV = 'd65a3688981405dc205f628f1a026bde8cf8303e'
PREFIX = 'examples/CtrBoard-H7_USB/'
ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'build' / 'dm_mc02' / 'sdk'

def fetch():
    DEST.mkdir(parents=True, exist_ok=True)
    manifest = DEST / 'tree.json'
    if not manifest.exists():
        url = f'https://api.github.com/repos/dmBots/DM-MC02/git/trees/{REV}?recursive=1'
        manifest.write_bytes(urllib.request.urlopen(url, timeout=60).read())
    tree = json.loads(manifest.read_text())
    if tree['sha'] != REV or tree.get('truncated'):
        raise RuntimeError('Incomplete or unexpected vendor manifest')
    files = [f for f in tree['tree'] if f['type']=='blob' and f['path'].startswith(PREFIX)
             and f['path'].endswith(('.c', '.h')) and '/vofa+' not in f['path'] and '/MDK-ARM/' not in f['path']
             and not any('/CMSIS/'+x+'/' in f['path'] for x in ('DSP','NN','Core_A','RTOS2'))]
    # USB example does not include the FDCAN HAL; use the same pinned CAN example.
    files += [f for f in tree['tree'] if f['type']=='blob' and
              f['path'].startswith('examples/CtrBoard-H7_CAN/Drivers/STM32H7xx_HAL_Driver/') and
              Path(f['path']).name in ('stm32h7xx_hal_fdcan.c','stm32h7xx_hal_fdcan.h')]
    def one(f):
        rel = f['path'].split('/',2)[2]
        path = DEST / rel
        if path.exists():
            data = path.read_bytes()
        else:
            url = f"https://raw.githubusercontent.com/dmBots/DM-MC02/{REV}/{f['path']}"
            data = urllib.request.urlopen(url, timeout=60).read()
        digest = hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if digest != f['sha']:
            raise RuntimeError(f'Vendor checksum mismatch: {path}')
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(one, files))
    print(f'Verified {len(files)} SDK files at {REV}')
if __name__ == '__main__':
    fetch()
