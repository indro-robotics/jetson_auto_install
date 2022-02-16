#!/bin/bash

test=google.com
while ! nc -zw1 $test 443 && echo |openssl s_client -connect $test:443 2>&1 |awk '
  handshake && $1 == "Verification" { if ($2=="OK") exit; exit 1 }
  $1 $2 == "SSLhandshake" { handshake = 1 }';
do
  echo "we don't have connectivity"
  sleep 1
done
echo "We are connected"
sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"
