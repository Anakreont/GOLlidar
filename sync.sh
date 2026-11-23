#!/bin/bash
# Kopia nagran z ~/skany na pendrive (PENDRIVE) i na serwer: przez HTTPS do BMS API (URL+TOKEN) albo rsync przez SSH (CEL). Uruchamiane co minute przez skaner-sync.timer.
# Konfiguracja: ~/skaner/sync.conf (wzor: sync.conf.example). Bez tego pliku skrypt nic nie robi.
CONF="$(dirname "$0")/sync.conf"
[ -f "$CONF" ] || exit 0
# shellcheck disable=SC1090
source "$CONF"
[ -n "$CEL" ] || [ -n "$URL" ] || [ -n "$PENDRIVE" ] || exit 0
ZRODLO="${ZRODLO:-$HOME/skany}"
STATUS="$ZRODLO/.sync_status"
mkdir -p "$ZRODLO"
# jedna kopia naraz
exec 9>"/tmp/skaner-sync.lock"
flock -n 9 || exit 0
# kopia na pendrive (bez sieci); PENDRIVE = punkt montowania, np. /media/pendrive
if [ -n "$PENDRIVE" ]; then
    PSTATUS="$ZRODLO/.sync_pendrive"
    if [ -n "$PENDRIVE_TEST" ] || mountpoint -q "$PENDRIVE"; then
        mkdir -p "$PENDRIVE/skany" &&
        rsync -rt --append-verify --modify-window=2 --exclude '.sync_*' "$ZRODLO/" "$PENDRIVE/skany/" && sync -f "$PENDRIVE/skany" \
            && echo "$(date +%s) OK" > "$PSTATUS" || echo "$(date +%s) BLAD" > "$PSTATUS"
    else
        echo "$(date +%s) BRAK" > "$PSTATUS"
    fi
fi
[ -n "$CEL" ] || [ -n "$URL" ] || exit 0
# tryb HTTPS (BMS API) ma pierwszenstwo przed rsync
if [ -n "$URL" ]; then
    if python3 "$(dirname "$0")/sync_http.py"; then echo "$(date +%s) OK" > "$STATUS"; exit 0
    else echo "$(date +%s) BLAD" > "$STATUS"; exit 1; fi
fi
if [ -n "$CEL_LOKALNY" ]; then
    RSH=()                                   # tryb testowy: kopia do katalogu lokalnego
else
    RSH=(-e "ssh -p ${PORT:-22} -o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=10 -o StrictHostKeyChecking=accept-new")
fi
# --append-verify: dla rosnacych plikow .bin wysylana jest tylko nowa koncowka
if rsync -a -z --append-verify --partial --timeout=40 --exclude '.sync_*' "${RSH[@]}" "$ZRODLO/" "$CEL"; then
    echo "$(date +%s) OK" > "$STATUS"
else
    echo "$(date +%s) BLAD" > "$STATUS"
    exit 1
fi
