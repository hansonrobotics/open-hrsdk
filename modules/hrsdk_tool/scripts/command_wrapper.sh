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

## For development please contact <dev@hansonrobotics.com>
##
##
#
# hrsdk
#
# Hanson Robotics Robot SDK Tool
#
set -e
BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
HRSDK_INSTALL_DIR=/opt/hansonrobotics/hrsdk

COLOR_INFO='\033[32m'
COLOR_WARN='\033[33m'
COLOR_ERROR='\033[31m'
COLOR_RESET='\033[0m'
info() {
    printf "${COLOR_INFO}${1}${COLOR_RESET}\n" >&2
}
warn() {
    printf "${COLOR_WARN}${1}${COLOR_RESET}\n" 1>&2
}
error() {
    printf "${COLOR_ERROR}${1}${COLOR_RESET}\n" 1>&2
}
fail() {
    printf "${COLOR_ERROR}${1}${COLOR_RESET}\n" 1>&2 && return 1
}

_hrsdk_start() {
    bash $HRSDK_INSTALL_DIR/start_sdk.sh $@
}

_hrsdk_stop() {
    bash $HRSDK_INSTALL_DIR/shutdown_sdk.sh $@
}

_hrsdk_pull() {
    bash $HRSDK_INSTALL_DIR/pull.sh $@
}

_hrsdk_update() {
    local arch=$(uname -m)
    if [[ $arch == 'aarch64' ]]; then
        arch='arm64'
    elif [[ $arch == 'x86_64' ]]; then
        arch='amd64'
    fi
    local url=https://s3.ap-northeast-2.amazonaws.com/dl.hansonrobotics.com/hrsdk-pkgs/head-hrsdk-tool_latest_${arch}.deb
    local tmp_deb=/tmp/head-hrsdk-tool.deb
    wget -q $url -O $tmp_deb && sudo dpkg -i $tmp_deb && sudo apt-get install -q -f && rm $tmp_deb
}

_hrsdk_push() {
    bash $HRSDK_INSTALL_DIR/push.sh $@
}

_hrsdk_init() {
    bash $HRSDK_INSTALL_DIR/init.sh $@
}

_hrsdk_login() {
    source $HRSDK_INSTALL_DIR/config.sh
    if [[ $# > 0 ]]; then
        AWS_REGION_NAME=${1}
    fi
    update_env
    login_ecr && info "Login $AWS_REGION_NAME successfully"
}

_hrsdk_comletion_bash() {
    cat $HRSDK_INSTALL_DIR/scripts/hrsdk-completion.bash
}

_hrsdk_upload() {
    bash $HRSDK_INSTALL_DIR/upload.sh $@
}

_hrsdk_servo_rw() {
    $HRSDK_INSTALL_DIR/scripts/rw $@
}

_hrsdk_servo_ping() {
    $HRSDK_INSTALL_DIR/scripts/ping $@
}


_hrsdk_comletion_services() {
    local services="asr bert blenderbot chatbot-server chatscript control control-hw duckling gpt2-server home intent-classifier interaction mysql nluserver opencog perception perception-advance rosmaster soultalk storage ttsserver"
    source $HRSDK_INSTALL_DIR/config.sh
    update_env
    get_config_schema
    case "$schema_number" in
        1.0)
            cat $CONFIG_PACKAGE_FILE | jq -r ".service_groups|.all[]" 2>/dev/null || true
            ;;
        1.1)
            local json_file=${VERSION_FILE}.json
            if [[ -f $json_file ]]; then
                cat $json_file | jq -r ".services|keys[]" 2>/dev/null || true
            fi
            ;;
        *)
            echo "${services[@]}"
            ;;
    esac
}

_hrsdk_comletion_groups() {
    local groups="default control hardware perception advance interaction"
    source $HRSDK_INSTALL_DIR/config.sh
    update_env
    get_config_schema
    case "$schema_number" in
        1.0)
            cat $CONFIG_PACKAGE_FILE | jq -r ".service_groups|keys[]" 2>/dev/null || true
            ;;
        1.1)
            local json_file=${VERSION_FILE}.json
            if [[ -f $json_file ]]; then
                cat $json_file | jq -r ".service_groups|keys[]" 2>/dev/null || true
            fi
            ;;
        *)
            echo "${groups[@]}"
            ;;
    esac
    echo full
}

_hrsdk_version() {
    dpkg-query -W -f='${Version}\n' head-hrsdk-tool
}

_decode_base64() {
    local len=$((${#1} % 4))
    local result="$1"
    if [ $len -eq 2 ]; then result="$1"'=='
        elif [ $len -eq 3 ]; then result="$1"'='
    fi
    echo "$result" | tr '_-' '/+' | openssl enc -d -base64
}

_get_storage_tag() {
    update_env
    if [[ ! -f $VERSION_FILE ]]; then
        generate_version_file ${VERSION_FILE}
    fi
    if [[ -f $VERSION_FILE ]]; then
        source $VERSION_FILE
    fi
    export HR_CONFIG_STORAGE=$HR_STORAGE/$CONFIG_NAME
    export LOCAL_STORAGE_PATH=/tmp/hrsdk_local_storage
    mkdir -p $LOCAL_STORAGE_PATH # a placeholder for local storage
    local tag=$($LAUNCHER $LAUNCHER_ARGS run --rm storage cat $HR_CONFIG_STORAGE/tag| grep "config hash" | cut -d: -f2 | xargs)
    echo $tag
}

_hrsdk_info() {
    source $HRSDK_INSTALL_DIR/config.sh

    local docker_root_dir=$(docker info | grep "Root Dir" | cut -d: -f2 | xargs)
    if [[ ! -z $CONFIG_NAME ]]; then
        local tag=$(_get_storage_tag)
    fi
    echo "Date: $(date)"
    echo "Host: $(hostname)"
    echo "OS: $(lsb_release -sd)"
    echo "Docker Root Dir: $docker_root_dir"

    local sdk_tool_version=$(dpkg-query -W -f='${Version}\n' head-hrsdk-tool)
    echo "SDK Tool Version: $sdk_tool_version"

    if [[ ! -z $HRSDK_VERSION ]]; then
        echo "SDK Version: $HRSDK_VERSION"
    fi
    if [[ ! -z $CONFIG_NAME ]]; then
        echo "Config name: $CONFIG_NAME"
        if [[ ! -z $tag ]]; then
            echo "Config tag: ${tag}"
        fi
    fi
    if [[ ! -z $REGISTRY_PREFIX ]]; then
        echo "Registry: $REGISTRY_PREFIX"
    fi
    if [[ ! -z $ENVIRONMENT ]]; then
        echo "Environment: $ENVIRONMENT"
    fi
    update_env
    get_config_schema
    local image images
    case "$schema_number" in
        1.1)
            echo "Image digests"
            images=( $(get_images_by_service_group full) )
            for image in ${images[@]}; do
                printf "  "
                created=$(docker image inspect "${image}" -f '{{.Created}}' 2>/dev/null || echo "")
                if [[ $created == "" ]]; then
                    warn "No image $image"
                    continue
                fi
                digest=$(docker image inspect "${image}" -f '{{.RepoDigests}}' 2>/dev/null || echo "")
                tag=$(echo ${image}| cut -d: -f2)
                if [[ $digest == "[]" ]]; then
                    echo "[${image}] @ $created => $tag"
                else
                    echo "${digest} @ $created => $tag"
                fi
            done
            ;;
        *)
            ;;
    esac

    echo "AWS Region: $AWS_REGION_NAME"
    identity=$(aws sts get-caller-identity | jq -r ".Arn" 2>/dev/null | cut -d/ -f2)
    echo "AWS User: $identity"
}

show_help() {
cat << EOF
Hanson Robotics Robot SDK Tool

Usage: hrsdk <command> [<args>]

  Supported commands:

    init
        Init Robot SDK
    start [options]
        Start Robot SDK
    stop
        Stop Robot SDK
    pull [options]
        Pull the files from Docker registry and S3
    push [options]
        Push the files to Docker registry and S3 [for admin]
    update
        Update the hrsdk CLI (this tool)
    upload [--logs] file1 [file2] [file3] ...
        Upload logs or files to S3
    completion bash
        Print the bash completion script
    login [region]
        Login the ECR with the region
    version
        Show the version of this hrsdk tool
    info
        Show the SDK info
    servo
        Allows to work with feetech servos. Use hrsdk servo ping, and hrsdk servo rw
EOF
}

execute() {
    case "$1" in
        start|stop|pull|push|init|login|version|info|update|upload)
            command=$1
            shift
            _hrsdk_${command} $@
            ;;
        completion)
            command=$2
            shift
            shift
            if [[ $command == "bash" ]]; then
                _hrsdk_comletion_bash
            elif [[ $command == "services" ]]; then
                _hrsdk_comletion_services
            elif [[ $command == "groups" ]]; then
                _hrsdk_comletion_groups
            else
                error "Unknown completion \"$command\""
                exit 1
            fi
            ;;
        servo)
            command=$2
            shift
            shift
            if [[ $command == "ping" ]]; then
                _hrsdk_servo_ping $@
            elif [[ $command == "rw" ]]; then
                _hrsdk_servo_rw $@
            else
                error "Unknown command. Use ping or rw to work with servos"
                exit 1
            fi
            ;;
        *)
            error "Unknown argument $1"
            show_help
            exit 1
            ;;
    esac
}

############# Main #############
if [[ ! $(readlink -f ${BASH_SOURCE[0]}) == $(readlink -f $0) ]]; then return; fi
if [[ $# == 0 ]]; then show_help; exit 0; fi

execute $@
