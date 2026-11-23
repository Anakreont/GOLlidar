#!/bin/bash
# Kopia nagran z ~/skany na serwer (rsync przez SSH). Uruchamiane co minute przez skaner-sync.timer.
# Konfiguracja: ~/skaner/sync.conf (wzor: sync.conf.example). Bez tego pliku skrypt nic nie robi.
CONF="$(dirname "$0")/sync.conf"
[ -f "$CONF" ] || exit 0
# shellcheck disable=SC1090
source "$CONF"
[ -n "$CEL" ] || exit 0
ZRODLO="${ZRODLO:-$HOME/skany}"
STATUS="$ZRODLO/.sync_status"
mkdir -p "$ZRODLO"
# jedna kopia naraz
exec 9>"/tmp/skaner-sync.lock"
flock -n 9 || exit 0
if [ -n "$CEL_LOKALNY" ]; then
    RSH=()                                   # tryb testowy: kopia do katalogu lokalnego
else
    RSH=(-e "ssh -p ${PORT:-22} -o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=10 -o StrictHostKeyChecking=accept-new")
fi
# --append-verify: dla rosnacych plikow .bin wysylana jest tylko nowa koncowka
if rsync -a -z --append-verify --partial --timeout=40 --exclude '.sync_status' "${RSH[@]}" "$ZRODLO/" "$CEL"; then
    echo "$(date +%s) OK" > "$STATUS"
else
    echo "$(date +%s) BLAD" > "$STATUS"
    exit 1
fi
