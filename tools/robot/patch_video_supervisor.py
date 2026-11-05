"""Small, checked edits to the inspected AOS camera script; no remote execution."""
from pathlib import Path
import sys


def patch(source):
    if '# M20_VIDEO_HEALTH_V1' in source:raise ValueError('already patched')
    def replace(old,new):
        nonlocal source
        if source.count(old)!=1:raise ValueError('unexpected camera script near: '+old[:70])
        source=source.replace(old,new,1)
    replace('LOG_TAG="[push_video]"','''LOG_TAG="[push_video]"
# M20_VIDEO_HEALTH_V1: local decoded-frame checks; no new background service.
VIDEO_HEALTH="$SCRIPT_DIR/video_health.py"
[ -f "$VIDEO_HEALTH" ] || { echo "Missing $VIDEO_HEALTH" >&2; exit 1; }''')
    replace('! jpegdec ! videoconvert ! mpph265enc',
            '! watchdog name=capture_watch timeout=8000 ! jpegdec ! videoconvert ! mpph265enc')
    replace('! h265parse config-interval=-1 ! rtspclientsink',
            '! h265parse config-interval=-1 ! watchdog name=encode_watch timeout=8000 ! rtspclientsink')
    replace('location=rtsp://127.0.0.1:8554/$stream_name latency=10',
            'location=rtsp://127.0.0.1:8554/$stream_name latency=10 protocols=tcp tcp-timeout=5000000')
    replace('eval "$pipeline" &',
            'python3 "$VIDEO_HEALTH" run "$stream_name" $pipeline &')
    replace('leftover=$(pgrep -f "gst-launch.*device=$current_dev" 2>/dev/null)',
            'leftover=$(python3 "$VIDEO_HEALTH" pids "$current_dev")')
    replace('leftover=$(pgrep -f "gst-launch.*device=$dev" 2>/dev/null)',
            'leftover=$(python3 "$VIDEO_HEALTH" pids "$dev")')
    # Global exit must not terminate unrelated GStreamer consumers.
    if source.count('pids=$(pgrep gst-launch 2>/dev/null)')!=2:raise ValueError('unexpected global cleanup')
    source=source.replace('pids=$(pgrep gst-launch 2>/dev/null)','pids=$(python3 "$VIDEO_HEALTH" pids)')
    replace('local stable_count=0','''local stable_count=0
        local health_failures=0 now started next_probe health_detail
        read -r started _ < /proc/uptime
        started=${started%.*}
        next_probe=$((started + 8))''')
    replace('''            stable_count=$((stable_count + 1))
            if [ "$stable_count" -eq "$STABLE_THRESHOLD" ]; then''','''            # Monotonic startup grace; a live PID/SDP does not establish video health.
            read -r now _ < /proc/uptime
            now=${now%.*}
            if [ "$now" -lt "$next_probe" ]; then
                continue
            fi
            next_probe=$((now + 8))
            if health_detail=$(python3 "$VIDEO_HEALTH" probe "$stream_name"); then
                health_failures=0
                stable_count=$((stable_count + 1))
            else
                health_failures=$((health_failures + 1))
                stable_count=0
                log_stream_error "$stream_name" "没有可解码新帧 ($health_failures/2): $health_detail"
                if [ "$health_failures" -ge 2 ]; then
                    log_stream_error "$stream_name" "流健康检查失败，重建本路；诊断见 /run/m20-video/$stream_name.log"
                    break
                fi
                continue
            fi
            if [ "$stable_count" -eq "$STABLE_THRESHOLD" ]; then''')
    replace('local delay=$((RETRY_BASE_DELAY * (2 ** (retry_count - 1))))','''local delay=$RETRY_MAX_DELAY
        if [ "$retry_count" -lt 5 ]; then
            delay=$((RETRY_BASE_DELAY * (2 ** (retry_count - 1))))
        fi''')
    return source


if __name__=='__main__':
    original,output=map(Path,sys.argv[1:]);output.write_text(patch(original.read_text()))
