"""Download NVIDIA wheels with TLS and published SHA-256 verification."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
from pathlib import Path
import shutil
import subprocess
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1] / '.cache/wheels'
FILES = [
 ('isaacsim-extscache-kit', 'isaacsim_extscache_kit-6.0.1.0-cp312-none-manylinux_2_35_x86_64.whl',
  5879615742, '35b64cf35ce3d31875ef43025f243c8bd03e29cb28fe844b0cd06b76ba535714'),
 ('isaacsim-extscache-kit-sdk', 'isaacsim_extscache_kit_sdk-6.0.1.0-cp312-none-manylinux_2_35_x86_64.whl',
  1686158517, 'a39c7a71f7a859e9db4ce6f81382f63a480d7e1938c6f3653702c9e6dd609c5e')]

def download(item):
    package, name, size, expected = item
    ROOT.mkdir(parents=True, exist_ok=True)
    output = ROOT / name
    chunk = 128 * 1024 * 1024
    count = (size + chunk - 1) // chunk
    url = f'https://pypi.nvidia.com/{package}/{name}'
    def fetch(index):
        start, end = index * chunk, min(size, (index + 1) * chunk) - 1
        part = ROOT / (name + f'.part{index:03}')
        if part.exists() and part.stat().st_size == end-start+1: return part
        for attempt in range(3):
            try:
                subprocess.run(['curl', '--fail', '--location', '--silent', '--show-error',
                    '--connect-timeout', '20', '--max-time', '180', '--retry', '2',
                    '--range', f'{start}-{end}', '--output', str(part), url], check=True)
                if part.stat().st_size != end-start+1: raise RuntimeError('Incomplete byte range')
                print(f'{package}: {index+1}/{count} chunks received', flush=True)
                return part
            except Exception:
                if attempt == 2: raise
                time.sleep(2)
    with ThreadPoolExecutor(max_workers=6) as executor: parts = list(executor.map(fetch, range(count)))
    digest = hashlib.sha256()
    temporary = output.with_suffix('.download')
    with temporary.open('wb') as target:
        for part in parts:
            with part.open('rb') as source:
                while block := source.read(1024*1024): target.write(block); digest.update(block)
    if digest.hexdigest() != expected: raise RuntimeError('SHA-256 mismatch: ' + name)
    temporary.replace(output)
    for part in parts: part.unlink()
    print('VERIFIED', output, flush=True)

if __name__ == '__main__':
    for item in FILES: download(item)
