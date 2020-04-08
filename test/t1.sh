#!/usr/bin/bash

set -euo pipefail

#
# UPDATE THE BELOW DEPENDING ON THE BOARD TYPE (uno or nano) AND ALSO UPDATE
# PORTS DEPENDING ON WHERE (ON WHICH DEV PATH) YOUR BOARD ARE PLUGGED.
#
BOARD0=nano
BOARD1=nano
PORT0=/dev/ttyUSB0
PORT1=/dev/ttyUSB1
AMEXE=./am

SND=../examples/example2/sender2/sender2.ino
RCV=../examples/example2/receiver2/receiver2.ino

OUT0=tmp0.out
OUT1=tmp1.out
REFOUT0=ref0.out
REFOUT1=ref1.out

echo "[S]"
"${AMEXE}" -b "${BOARD0}" -p "${PORT0}" "${SND}"
echo "[R]"
"${AMEXE}" -b "${BOARD1}" -p "${PORT1}" "${RCV}"

echo ""
echo "[S]"
"${AMEXE}" -b "${BOARD0}" -p "${PORT0}" "${SND}" -n -u
echo "[R]"
"${AMEXE}" -b "${BOARD1}" -p "${PORT1}" "${RCV}" -n -u

echo ""
CMD=timeout
CMD_OPTS="9 "${AMEXE}""
#CMD="${AMEXE}"
#CMD_OPTS=
"${CMD}" ${CMD_OPTS} -p "${PORT0}" "${SND}" -n -c -r \
    --recordfile "${OUT0}" &
"${CMD}" ${CMD_OPTS} -p "${PORT1}" "${RCV}" -n -c -r \
    --recordfile "${OUT1}" &

set +e

AMEXE_BASE=$(basename "${AMEXE}")
while pgrep "\<${AMEXE_BASE}\>" > /dev/null; do
    sleep 1
done

for f in "${OUT0}" "${OUT1}"; do
    sed -n '/^-----BEGIN ARDUINO OUTPUT-----/,/^-----END ARDUINO OUTPUT-----/p' \
        "${f}" > "${f}.2"
done

if cmp -s "${OUT0}.2" "${REFOUT0}"; then
    echo "   0: Ok"
else
    echo "** 0: output does not match!"
fi

if cmp -s "${OUT1}.2" "${REFOUT1}"; then
    echo "   1: Ok"
else
    echo "** 1: output does not match!"
fi

