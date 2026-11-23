#!/bin/bash
# Uruchamiane na serwerze: pobiera nagrania ze skanera przez tunel Cloudflare (SSH) i dopisuje je do kopii.
# Konfiguracja w /etc/lidar-pobierz.conf:
#   HOST="ssh-NAZWA.twoja-domena"      # adres SSH skanera w tunelu
#   UZYTKOWNIK="anakreont"
#   CEL="/sciezka/na/nagrania"
set -u
. /etc/lidar-pobierz.conf || exit 1
UZYTKOWNIK="${UZYTKOWNIK:-anakreont}"
exec 9>/tmp/lidar-pobierz.lock
flock -n 9 || exit 0
mkdir -p "$CEL"
SSH="ssh -o ProxyCommand='cloudflared access ssh --hostname %h' -o BatchMode=yes -o ConnectTimeout=20 -o ServerAliveInterval=15 -o ServerAliveCountMax=3"
if rsync -a -z --append-verify --partial --timeout=45 --exclude=.sync_status -e "$SSH" "$UZYTKOWNIK@$HOST:skany/" "$CEL/"; then
    # potwierdzenie dla strony skanera (kafelek "Kopia na serwer")
    eval "$SSH" "$UZYTKOWNIK@$HOST" "'echo \$(date +%s) OK > ~/skany/.sync_status'"
    exit 0
fi
exit 1
