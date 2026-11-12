"""Read primary research pages for the assessment; no simulator work."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path
import re
import urllib.request

SOURCES = [
    ('dexart', 'https://arxiv.org/abs/2305.05706'),
    ('dexgraspnet', 'https://arxiv.org/abs/2210.02697'),
    ('oakink2', 'https://oakink.net/v2'),
]

def fetch(item):
    key, url = item
    record = {'id': key, 'requested_url': url}
    try:
        request = urllib.request.Request(url, headers={'User-Agent': 'BenchmarkEvidenceAudit/1.0'})
        with urllib.request.urlopen(request, timeout=25) as response:
            raw = response.read()
            text = raw.decode('utf-8', errors='replace')
            title = re.search(r'<title[^>]*>(.*?)</title>', text, re.S | re.I)
            text = re.sub(r'(?is)<(script|style).*?</\1>', '', text)
            text = re.sub(r'<[^>]+>', ' ', text)
            text = re.sub(r'\s+', ' ', html.unescape(text)).strip()
            record.update(status=response.status, final_url=response.url,
                          title=html.unescape(title.group(1)).strip() if title else None,
                          retrieved_utc=datetime.now(timezone.utc).isoformat(),
                          response_sha256=hashlib.sha256(raw).hexdigest(), extracted_text=text)
    except Exception as exc:
        record.update(status='FETCH_FAILED', error=f'{type(exc).__name__}: {exc}')
    return record

if __name__ == '__main__':
    with ThreadPoolExecutor(max_workers=3) as pool:
        records = list(pool.map(fetch, SOURCES))
    output = Path(__file__).with_name('functional_reference_sources_20261030.json')
    with output.open('x') as stream:
        json.dump({'sources': records, 'scope': 'Selected primary-source comparisons, not exhaustive literature search'}, stream, ensure_ascii=False, indent=2)
    for record in records:
        print(json.dumps(record, ensure_ascii=False))
