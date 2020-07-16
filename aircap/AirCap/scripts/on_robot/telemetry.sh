#!/bin/sh

mydate=$( date +%s )

../librepilot-mpg/ground/TelemetryBridge/bridge >${LOGDIR}/current/telemetry_${mydate}.opl

echo "telemetry failed with errorcode $?" >>${LOGDIR}/current/faillog
