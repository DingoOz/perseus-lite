#!/usr/bin/env bash
# One-shot clock step from the HTTPS `Date` header of a well-known server.
#
# For robots without an RTC battery / NTP access (eg NTP blocked on a
# competition or campus network). Installed to /usr/local/sbin by
# timesync-google-setup.sh and run by timesync-google.service.
#
# Safety rules:
#   - the header is fetched over HTTPS with a timeout and retries; TLS
#     certificate validation is only skipped when the local clock is so wrong
#     (year < MIN_TRUSTED_YEAR) that validation cannot possibly succeed,
#   - the Date value must be non-empty, parseable and itself plausible,
#   - the clock is never stepped backwards by more than MAX_BACKSTEP_S unless
#     the local clock is unsynced (year < MIN_TRUSTED_YEAR),
#   - any failure exits non-zero, so the unit shows as failed instead of
#     silently "succeeding" (the old `date -s ""` set the clock to midnight).
#
# Overridable via environment: TIMESYNC_URL, TIMESYNC_MAX_BACKSTEP_S,
# TIMESYNC_TIMEOUT_S. TIMESYNC_DRY_RUN=1 prints the decision without setting
# the clock (handy for testing as a normal user).

set -euo pipefail

URL="${TIMESYNC_URL:-https://www.google.com}"
MAX_BACKSTEP_S="${TIMESYNC_MAX_BACKSTEP_S:-3600}"
TIMEOUT_S="${TIMESYNC_TIMEOUT_S:-15}"
DRY_RUN="${TIMESYNC_DRY_RUN:-0}"
MIN_TRUSTED_YEAR=2024

log() { echo "timesync-google: $*" >&2; }

# Prints the raw response headers of a HEAD request to $URL.
# $1 = "verify" or "insecure".
fetch_headers() {
  local mode="$1"
  if command -v curl >/dev/null 2>&1; then
    local -a tls=()
    [[ $mode == insecure ]] && tls=(--insecure)
    curl --silent --show-error --head "${tls[@]}" \
      --max-time "$TIMEOUT_S" --retry 3 --retry-delay 2 --retry-connrefused \
      "$URL"
  elif command -v wget >/dev/null 2>&1; then
    local -a tls=()
    [[ $mode == insecure ]] && tls=(--no-check-certificate)
    # wget writes the server response (-S) to stderr
    wget --method=HEAD --server-response --quiet --output-document=/dev/null \
      "${tls[@]}" --timeout="$TIMEOUT_S" --tries=3 --waitretry=2 \
      "$URL" 2>&1
  else
    log "neither curl nor wget is installed"
    return 1
  fi
}

current_year="$(date -u +%Y)"
clock_unsynced=0
((current_year < MIN_TRUSTED_YEAR)) && clock_unsynced=1

if ! headers="$(fetch_headers verify)"; then
  if ((clock_unsynced)); then
    log "verified HTTPS fetch failed with local year ${current_year}; retrying without certificate validation"
    headers="$(fetch_headers insecure)" || {
      log "could not fetch headers from $URL"
      exit 1
    }
  else
    log "could not fetch headers from $URL"
    exit 1
  fi
fi

# Last Date header wins (redirect chains print several header blocks).
date_header="$(printf '%s\n' "$headers" | tr -d '\r' |
  sed -n 's/^[[:space:]]*[Dd][Aa][Tt][Ee]:[[:space:]]*//p' | tail -n 1)"
if [[ -z $date_header ]]; then
  log "no Date header in response from $URL"
  exit 1
fi

if ! remote_epoch="$(date -u -d "$date_header" +%s 2>/dev/null)"; then
  log "unparsable Date header: '$date_header'"
  exit 1
fi
remote_year="$(date -u -d "@$remote_epoch" +%Y)"
if ((remote_year < MIN_TRUSTED_YEAR)); then
  log "implausible Date header '$date_header' (year ${remote_year})"
  exit 1
fi

local_epoch="$(date -u +%s)"
delta=$((remote_epoch - local_epoch))

if ((delta < -MAX_BACKSTEP_S)) && ((!clock_unsynced)); then
  log "refusing to step clock back by $((-delta))s (limit ${MAX_BACKSTEP_S}s); remote says '$date_header'"
  exit 1
fi

if ((delta >= -1 && delta <= 1)); then
  log "clock already within 1s of $URL (delta ${delta}s); not stepping"
  exit 0
fi

if [[ $DRY_RUN == 1 ]]; then
  log "dry run: would step clock by ${delta}s to '$date_header'"
  exit 0
fi

date -u -s "@$remote_epoch" >/dev/null
log "stepped clock by ${delta}s to $(date -u '+%Y-%m-%d %H:%M:%S UTC')"
