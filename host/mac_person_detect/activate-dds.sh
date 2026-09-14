#!/usr/bin/env bash
# Pixi / run-script: FastDDS peers = multicast + localhost + both Pi unicast.
# A list with only a Pi unicast peer replaces default multicast, so two Mac
# processes (detector and echo) never see each other. Do not disable builtin
# transports (custom UDP + interfaceWhiteList segfaults rmw_fastrtps here).
_DDS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_DDS_ETH="192.168.88.56"
_DDS_WIFI="192.168.149.1"
_DDS_DOMAIN="${ROS_DOMAIN_ID:-1}"
_DDS_MC_PORT=$((7400 + 250 * _DDS_DOMAIN))
_DDS_PEER_PORT=$((7400 + 250 * _DDS_DOMAIN + 10))

_DDS_LOCAL_PEERS=""
_p="${_DDS_PEER_PORT}"
_i=0
while [[ "${_i}" -lt 32 ]]; do
  _DDS_LOCAL_PEERS="${_DDS_LOCAL_PEERS}
          <locator>
            <udpv4>
              <address>127.0.0.1</address>
              <port>${_p}</port>
            </udpv4>
          </locator>"
  _p=$((_p + 2))
  _i=$((_i + 1))
done

_DDS_UNICAST_PEERS="
          <locator>
            <udpv4>
              <address>${_DDS_ETH}</address>
              <port>${_DDS_PEER_PORT}</port>
            </udpv4>
          </locator>
          <locator>
            <udpv4>
              <address>${_DDS_WIFI}</address>
              <port>${_DDS_PEER_PORT}</port>
            </udpv4>
          </locator>"
if [[ -n "${PI_DDS_PEER:-}" && "${PI_DDS_PEER}" != "${_DDS_ETH}" && "${PI_DDS_PEER}" != "${_DDS_WIFI}" ]]; then
  _DDS_UNICAST_PEERS="${_DDS_UNICAST_PEERS}
          <locator>
            <udpv4>
              <address>${PI_DDS_PEER}</address>
              <port>${_DDS_PEER_PORT}</port>
            </udpv4>
          </locator>"
fi

cat > "${_DDS_DIR}/fastdds-lan.xml" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="person_detect_lan" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>239.255.0.1</address>
              <port>${_DDS_MC_PORT}</port>
            </udpv4>
          </locator>
          ${_DDS_LOCAL_PEERS}${_DDS_UNICAST_PEERS}
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
EOF

export FASTRTPS_DEFAULT_PROFILES_FILE="${_DDS_DIR}/fastdds-lan.xml"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
_DDS_LOG="FastDDS peers ${_DDS_ETH}:${_DDS_PEER_PORT} ${_DDS_WIFI}:${_DDS_PEER_PORT}"
if [[ -n "${PI_DDS_PEER:-}" && "${PI_DDS_PEER}" != "${_DDS_ETH}" && "${PI_DDS_PEER}" != "${_DDS_WIFI}" ]]; then
  _DDS_LOG="${_DDS_LOG} ${PI_DDS_PEER}:${_DDS_PEER_PORT}"
fi
echo "${_DDS_LOG} multicast 239.255.0.1:${_DDS_MC_PORT} domain ${_DDS_DOMAIN}" >&2
ros2 daemon stop >/dev/null 2>&1 || true
unset _DDS_DIR _DDS_ETH _DDS_WIFI _DDS_DOMAIN _DDS_MC_PORT _DDS_PEER_PORT \
  _DDS_LOCAL_PEERS _DDS_UNICAST_PEERS _DDS_LOG _p _i
