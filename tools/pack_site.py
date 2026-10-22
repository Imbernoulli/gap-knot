"""Make the results page a single HTML file with embedded images and videos."""
import argparse
import base64
import mimetypes
import re
from pathlib import Path


def pack(source: Path, output: Path):
    def embed(match):
        path = source.parent / match[2]
        mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        data = base64.b64encode(path.read_bytes()).decode("ascii")
        return f'{match[1]}="data:{mime};base64,{data}"'

    html = re.sub(r'(src|poster)="(media/[^\"]+)"', embed, source.read_text())
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html)
    print(output)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[1] / "site/index.html")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    pack(args.source, args.out)
