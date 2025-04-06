#!/usr/bin/env bash

##
## Copyright (C) 2017-2025 Hanson Robotics
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.
##

set -e

show_help() {
cat << EOF

Usage: hrsdk start [--group] [--service] [--storage <directory>] [--env]
                   [--stt] [--gui]
                   [--local_ip <IP>] [--master_ip <IP>]
                   [--head] [--body]
                   [--pull <type>]
                   [--project <project>]
                   [-p] [-c] [-i] [--hw]

    --group
        start service group
        use multiple --group options to start multiple service groups
    --service
        start the service
        use multiple --service options to start multiple services
    --storage <directory>
        the directory for the local storage
    --env
        set the enviornment variable
        use multiple --env options to set multiple enviornment variables
    --stt
        enable google cloud speech
    --gui:
        enable UI (for blender)
    --dev:
        run in dev mode
    --local_ip <IP>
        the IP address for the local machine
    --master_ip <IP>
        the IP address for the machine runs ROS master
    --head
        the name of the robot head
    --body
        the name of the robot body
    --pull <type>
        pull the content, models etc
    --project <project>
        specify the project of the services (default project: hrsdk)
    -p:
        start perception container (DEPRECATED)
    -c:
        start control container (DEPRECATED)
    -i:
        start interaction container (DEPRECATED)
    --hw:
        start hardware control container (DEPRECATED)

EOF
}

parse_args() {
    while [[ $# > 0 ]]; do
        case "$1" in
            -p)
                service_group_names+=(perception)
                shift
                ;;
            --advance)
                service_group_names+=(advance)
                shift
                ;;
            -c)
                service_group_names+=(control)
                shift
                ;;
            --hw)
                service_group_names+=(hardware)
                export ENABLE_HARDWARE=1
                shift
                ;;
            -i)
                service_group_names+=(interaction)
                shift
                ;;
            --gui)
                export ENABLE_UI=1
                shift
                ;;
            --dev)
                export DEV_MODE=1
                info "Run in dev mode"
                shift
                ;;
            --master_ip)
                REMOTE=1
                ip=$2
                [[ -z $ip ]] && error "--master_ip requires IP address" && exit 1
                export ROS_MASTER_URI=http://$2:11311
                shift
                shift
                ;;
            --local_ip)
                ip=$2
                [[ -z $ip ]] && error "--local_ip requires IP address" && exit 1
                export ROS_IP=$2
                shift
                shift
                ;;
            --stt)
                ENABLE_GOOGLE_SPEECH=1
                shift
                ;;
            --storage)
                export LOCAL_STORAGE_PATH=$2
                if [[ ! -d $LOCAL_STORAGE_PATH ]]; then
                    error "Local storage \"$LOCAL_STORAGE_PATH\" doesn't exist"
                    exit 1
                fi
                if [[ $LOCAL_STORAGE_PATH != /* ]]; then
                    # convert to absolute path
                    LOCAL_STORAGE_PATH=$(realpath $LOCAL_STORAGE_PATH)
                fi
                export USING_LOCAL_STORAGE=1
                shift
                shift
                ;;
            --head)
                export ROBOT_NAME=$2
                shift
                shift
                ;;
            --body)
                export ROBOT_BODY=$2
                shift
                shift
                ;;
            --pull)
                pull_type=$2
                case $pull_type in
                    content)
                        export PULL_CHAT_CONTENT=1
                        shift
                        ;;
                    content-only)
                        export PULL_CHAT_CONTENT=1
                        export PULL_ONLY=1
                        shift
                        ;;
                    *)
                        fail "Unknown pull type \"$pull_type\". Type can be: content, content-only"
                        ;;
                esac
                shift
                ;;
            --service)
                customized_services+=($2)
                shift
                shift
                ;;
            --env)
                env_vars+=($2)
                shift
                shift
                ;;
            --group)
                service_group_names+=($2)
                shift
                shift
                ;;
            --project)
                export HR_PROJECT=$2
                shift
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                error "Unknown option $1"
                show_help
                exit 1
                ;;
        esac
    done

    update_env # should run after the parse_args
}

_set_services() {
    if [[ ${#service_group_names[@]} == 0 && ${#customized_services[@]} == 0 ]]; then
        IFS=' ' read -r -a default_groups <<< "${DEFAULT_SERVICE_GROUPS:-default}"
        service_group_names+=${default_groups[@]}
    fi
    local group_services=()
    local _group_services
    local gpu_services=()

    get_config_schema
    info "Schema $schema_number"
    case "$schema_number" in
        0.5)
            fail "The SDK tool does not support this config"
            ;;
        0.6)
            if [[ -f $SERVICE_CONFIG_FILE ]]; then
                # old service config in services.txt
                for group_name in ${service_group_names[@]}; do
                    _group_services=($(sed -n "/^\s*${group_name}\s*:/p" $SERVICE_CONFIG_FILE |cut -d: -f2))
                    if [[ -z $_group_services ]]; then
                        fail "Service group ${group_name} was not found"
                    fi
                    group_services+=( ${_group_services[@]} )
                done
                gpu_services=($(sed -n "/^\s*gpu_services\s*:/p" $SERVICE_CONFIG_FILE |cut -d: -f2))
            else
                fail "No service config file"
            fi
            ;;
        1.0)
            # new service config in package.json
            for group_name in ${service_group_names[@]}; do
                _group_services=$(cat $CONFIG_PACKAGE_FILE | jq -r '.service_groups["'"${group_name}"'"][]' 2>/dev/null)
                if [[ -z $_group_services ]]; then
                    fail "Service group ${group_name} was not found"
                fi
                group_services+=( ${_group_services[@]} )
            done
            gpu_services=$(cat $CONFIG_PACKAGE_FILE | jq -r ".service_groups.gpu_services|.[]" 2>/dev/null)
            ;;
        1.1)
            # service groups moved to version json file
            local json_file=${VERSION_FILE}.json
            for group_name in ${service_group_names[@]}; do
                _group_services=$(cat $json_file | jq -r '.service_groups["'"${group_name}"'"][]' 2>/dev/null)
                if [[ -z $_group_services ]]; then
                    fail "Service group ${group_name} was not found"
                fi
                group_services+=( ${_group_services[@]} )
            done
            gpu_services=($(cat $json_file | jq -r '.service_groups.gpu_services|.[]' 2>/dev/null))
            ;;
        *)
            fail "Unknown config schema $schema_number"
            ;;
    esac

    services+=( ${group_services[@]} )
    services+=( ${customized_services[@]} )

    # remove the GPU services if GPU is not available
    if [[ $NVIDIA_RUNTIME != 1 ]]; then
        # remove the GPU sevices
        for target in "${gpu_services[@]}"; do
            for i in "${!services[@]}"; do
                if [[ ${services[i]} == $target ]]; then
                    warn "Cannot run \"${services[i]}\" service: Nvidia runtime was not found."
                    unset 'services[i]'
                fi
            done
        done
    fi

    if [[ $REMOTE == 1 ]]; then
        # remove rosmaster as only one rosmaster can exist
        info "removing rosmaster"
        for i in "${!services[@]}"; do
            if [[ ${services[i]} == rosmaster ]]; then
                unset 'services[i]'
            fi
        done
    fi

    if [[ $PULL_ONLY == 1 ]]; then
        info "pull only"
        services=( storage )
    fi
    if [[ $PULL_CHAT_CONTENT == 1 ]]; then
        info "adding storage"
        services+=( storage )
    else
        info "no storage"
    fi

    # remove duplicted services
    services=( $(printf '%s\n' "${services[@]}" | sort -u) )

    if (( ${#services[@]} == 0 )); then
        fail "No services to run"
    fi
    info "Services: ${services[@]}"
}

_detect_realsense_camera() {
    shopt -s nullglob
    for dev in /dev/video*; do
        if [[ -c $dev ]]; then
            for link in $(udevadm info -q symlink $dev); do
                if [[ $link =~ .*RealSense.* ]]; then
                    local camera=/dev/${link}
                    local ID_MODEL=$(udevadm info -q all $camera |grep ID_MODEL=|cut -d= -f2)
                    export REALSENSE_DETECTED=1
                fi
            done
        fi
    done
    if [[ $REALSENSE_DETECTED == 1 ]]; then
        info "RealSense camera detected"
    fi
}

_lock() {
    if mkdir $LOCKDIR; then
        trap "sdk_shutdown" EXIT SIGINT SIGTERM ERR
    else
        error "Could not create lock directory '$LOCKDIR'. The process may have started"
        error "Run \"hrsdk stop\" to shut it down"
        exit 1
    fi
}

_check_ros_ip() {
    # Check if ROS_IP is set, if not no need to wait
    if [[ -z "$ROS_IP" ]]; then
        return 0
    fi
    local ip_address="$ROS_IP"
    local timeout="$1"
    local start_time=$(date +%s)

    info "Waiting for ROS IP ($ip_address) to become available..."

    while true; do
        # Check if the IP is pingable
        if ip addr show up|grep "$ip_address" &> /dev/null; then
            return 0
        fi
        # Check for timeout
        local current_time=$(date +%s)
        if (( current_time - start_time >= timeout )); then
            error "Local ROS_IP ($ip_address) not assigned to this. Will use localhost instead. Please check your network settings."
            export ROS_IP="127.0.0.1"
            return 0
        fi
        # Wait for a short period before trying again
        sleep 1
    done
}

main() {
    HRSDK_INSTALL_DIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
    NVIDIA_RUNTIME=$(docker info |grep Runtimes |grep nvidia|wc -l)
    source $HRSDK_INSTALL_DIR/config.sh

    services=()
    customized_services=()
    service_group_names=()
    parse_args $@

    if [[ $USING_LOCAL_STORAGE == 1 ]]; then
        warn "Using local storage $LOCAL_STORAGE_PATH"
        if hash git >/dev/null 2>&1; then
            local commit=$(git -C ${LOCAL_STORAGE_PATH} rev-parse --short HEAD)
            local branch=$(git -C ${LOCAL_STORAGE_PATH} rev-parse --abbrev-ref HEAD)
            warn "Storage head: $commit ($branch)"
        fi
    fi
    check_version

    if [[ ! -z $REGISTRY_PREFIX && $REGISTRY_PREFIX != default ]]; then
        warn "Registry Prefix: $REGISTRY_PREFIX"
    fi

    if [[ ! -f $LAUNCH_FILE ]]; then
        fail "Couldn't find main launch file. Please run \"hrsdk pull\""
    fi

    if [[ $USING_LOCAL_STORAGE != 1 ]]; then
        if [[ -z $CONFIG_NAME ]]; then
            error "CONFIG_NAME is not set"
            exit 1
        fi
        info "Config name: $CONFIG_NAME"
    fi

    if [[ -z $ROBOT_NAME || -z $ROBOT_BODY ]]; then
        detect_hw
    fi

    update_tablet_robot

    _detect_realsense_camera

    # load robot specific env before set services
    local body_env=$HW_CONFIGS_DIR/bodies/$ROBOT_BODY/env.sh
    local head_env=$HW_CONFIGS_DIR/heads/$ROBOT_NAME/env.sh
    if [[ ! -z $ROBOT_BODY && -f $body_env ]] ; then
        source $body_env
    fi
    if [[ ! -z $ROBOT_NAME && -f $head_env ]]; then
        source $head_env
    fi

    _set_services

    info "Starting SDK: $HRSDK_VERSION"
    info "Name: ${ROBOT_NAME}"
    info "Body: ${ROBOT_BODY}"

    if [[ " ${services[@]} " =~ " control " || \
        " ${services[@]} " =~ " perception " || \
        " ${services[@]} " =~ " perception-advance " || \
        " ${services[@]} " =~ " dev " || \
        " ${services[@]} " =~ " asr " ]]; then
        if grep -qi "microsoft" /proc/version &>/dev/null; then
            info "WSL detected"
            if [[ -n "$PULSE_SERVER" ]]; then
                export PULSE_SOCKET_PATH=$(printenv PULSE_SERVER | sed 's/^unix://')
                export PULSE_SERVER_DIR=$(dirname $PULSE_SOCKET_PATH)
            el
                warn "PULSE_SERVER environment variable is not set. Audio may not work."
            fi
        elif [[ $(lsb_release -rs) < "24.04" ]]; then
            info "Loading audio module"
            # https://wiki.archlinux.org/index.php/PulseAudio/Examples#Allowing_multiple_users_to_use_PulseAudio_at_the_same_time
            rm -f /tmp/pulseaudio.socket && pacmd load-module module-native-protocol-unix auth-anonymous=1 socket=/tmp/pulseaudio.socket || warn "Loading audio module failed. Audio may not be working"
        else
            info "Pulseaudio socket path: /run/user/${UID:-1000}/pulse/native"
            export PULSE_SERVER_DIR=/run/user/${UID:-1000}/pulse
            export PULSE_SOCKET_PATH=/run/user/${UID:-1000}/pulse/native
        fi

    fi

    if [[ $ENABLE_UI == 1 ]]; then
        $HRSDK_INSTALL_DIR/x_auth >/dev/null
    fi
    touch $BLENDER_RIG_FILE  # so it can be mounted to container

    if [[ -z $GOOGLE_APPLICATION_CREDENTIALS ]]; then
        export GOOGLE_APPLICATION_CREDENTIALS=/tmp/google_speech_not_set.json
    fi
    touch $GOOGLE_APPLICATION_CREDENTIALS

    # export enviorment variables handle
    for var in ${env_vars[@]}; do
        export $var
    done

    if [[ $ENABLE_GOOGLE_SPEECH == 1 ]]; then
        export GOOGLE_SPEECH=1
    fi
    # Check ROS IP. Sometimes router takes longer that SDK to autostart
    _check_ros_ip 90
    if [[ ${#services[@]} != 0 ]]; then
        if [[ " ${services[@]} " =~ " rosmaster " ]]; then
            _lock
        fi
        generate_version_file ${VERSION_FILE}
        source $VERSION_FILE
        $LAUNCHER $LAUNCHER_ARGS up --remove-orphans "${services[@]}"
    fi
}

main $@
