#!/bin/sh
# Rebuilds the three driver control sheets in docs/ from driver-controls.html, using Chrome.
set -e
cd "$(dirname "$0")"
CHROME="${CHROME:-/Applications/Google Chrome.app/Contents/MacOS/Google Chrome}"
SRC="file://$(pwd)/driver-controls.html"
for v in xbox ps4 ps5; do
  case $v in
    xbox) out=../Starter-Bot-Driver-Controls.pdf ;;
    ps4)  out=../Starter-Bot-Driver-Controls-PS4.pdf ;;
    ps5)  out=../Starter-Bot-Driver-Controls-PS5.pdf ;;
  esac
  "$CHROME" --headless --disable-gpu --no-pdf-header-footer --virtual-time-budget=2000 \
    --print-to-pdf="$out" "$SRC#$v" 2>/dev/null
  echo "wrote docs/${out#../}"
done
