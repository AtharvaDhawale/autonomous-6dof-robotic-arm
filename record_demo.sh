#!/usr/bin/env bash
# ==============================================================================
# Screen Recording & High-Resolution GIF Converter for Robotic Arm Demos
# Captures the active X11 screen and generates an optimized, lightweight GIF.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MEDIA_DIR="$SCRIPT_DIR/media"
mkdir -p "$MEDIA_DIR"

RAW_VIDEO="$SCRIPT_DIR/demo_raw.mp4"
PALETTE="/tmp/palette.png"
OUTPUT_GIF="$MEDIA_DIR/demo.gif"

# Verify ffmpeg
if ! command -v ffmpeg &>/dev/null; then
    echo "[ERROR] ffmpeg is not installed on this system."
    echo "[INFO] Install it with: sudo apt update && sudo apt install -y ffmpeg"
    exit 1
fi

# Detect display and resolution
DISP="${DISPLAY:-:0.0}"

if command -v xdpyinfo &>/dev/null; then
    RES=$(xdpyinfo | awk '/dimensions:/ { print $2 }')
else
    RES="1920x1080"
fi
echo "[INFO] Recording Display: $DISP at resolution: $RES"
echo "[INFO] Raw video will be saved to: $RAW_VIDEO"
echo "[INFO] Press 'q' or 'Ctrl+C' in this terminal when the demo completes..."
echo ""

# Record X11 display at 30 FPS
ffmpeg -y \
    -video_size "$RES" \
    -framerate 30 \
    -f x11grab \
    -i "$DISP" \
    -c:v libx264 \
    -preset ultrafast \
    -pix_fmt yuv420p \
    "$RAW_VIDEO" || true

if [ ! -f "$RAW_VIDEO" ]; then
    echo "[ERROR] Recording failed or was aborted before any frames were written."
    exit 1
fi

echo ""
echo "=============================================================================="
echo " CONVERTING RAW VIDEO TO OPTIMIZED GIF (TWO-PASS PALETTE GENERATION)"
echo "=============================================================================="

# Pass 1: Generate optimal color palette
echo "[1/2] Generating color palette..."
ffmpeg -y -i "$RAW_VIDEO" -vf "fps=15,scale=960:-1:flags=lanczos,palettegen" "$PALETTE"

# Pass 2: Apply palette for crystal-clear, lightweight GIF
echo "[2/2] Rendering GIF with palette application..."
ffmpeg -y -i "$RAW_VIDEO" -i "$PALETTE" -filter_complex "fps=15,scale=960:-1:flags=lanczos[x];[x][1:v]paletteuse" "$OUTPUT_GIF"

rm -f "$PALETTE"

echo ""
echo "[SUCCESS] Demo GIF generated at: $OUTPUT_GIF"
ls -lh "$OUTPUT_GIF"
